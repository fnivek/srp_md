#include <srp_md/factor_graph_worker.h>

FactorGraphWorker::FactorGraphWorker()
{
}

void FactorGraphWorker::Setup()
{
    ros::NodeHandle nh;
    goal_server_ = nh.advertiseService("get_goal", &FactorGraphWorker::GetGoal, this);
}

bool FactorGraphWorker::GetGoal(srp_md_msgs::GetGoalRequest& req, srp_md_msgs::GetGoalResponse& resp)
{
    // Build scene graph from objects
    dai::sceneGraph scene_graph;

    // Add all object vars
    for (int i = 0; i < req.objects.size(); ++i)
    {
        // Choose the correct class
        dai::ObjectClass cls;
        switch (req.classes[i])
        {
            case srp_md_msgs::GetGoalRequest::CLASS_CONTAINER:
                cls = dai::ObjectClass::kContainer;
                break;
            case srp_md_msgs::GetGoalRequest::CLASS_SUPPORTER:
                cls = dai::ObjectClass::kSupporter;
                break;
            case srp_md_msgs::GetGoalRequest::CLASS_PROP:
            default:
                cls = dai::ObjectClass::kProp;
                break;
        }
        scene_graph.addObjectVar(req.objects[i], cls, req.num_states[i]);
    }

    // Generate all relationships between objects
    scene_graph.generateRelationVars();

    // Make all the learned factors
    for (const auto& ros_factor : req.factors)
    {
        dai::VarSet sorted_vars;
        std::vector<dai::Var> vars;
        // Get the Vars for singular objects
        for (const auto& obj_name : ros_factor.objects)
        {
            dai::Var v = scene_graph.getObjectVarByName(obj_name);
            vars.push_back(v);
            sorted_vars.insert(v);
        }
        // Get the Vars for pairs of objects
        for (const auto& pair : ros_factor.pairs)
        {
            dai::ObjectPair v;
            if (!scene_graph.getRelationVarByNames(pair.object1, pair.object2, &v))
                printf("ERROR: pair {%s, %s} is in a different order than the scene graph\n", pair.object1.c_str(),
                       pair.object2.c_str());
            vars.push_back(v);
            sorted_vars.insert(v);
        }
        // Re-order ros_factor.probs to match the ordering of vars
        // Assumption is that all objects are first in the order they are in the ros_factor.objects then all relation
        // vars in order of ros_factor.pairs
        std::vector<dai::Real> sorted_probs;
        reorderFactorProbs(vars, ros_factor.probs, &sorted_probs);
        scene_graph.addFactor(sorted_vars, sorted_probs);
    }

    // Choose which priors to use
    bool use_consistency = false;
    bool use_commensense = false;
    bool use_no_float = false;
    for (auto& prior : req.prior_knowledge)
    {
        switch (prior)
        {
            case srp_md_msgs::GetGoalRequest::CONSISTENCY_PRIOR:
                use_consistency = true;
                break;
            case srp_md_msgs::GetGoalRequest::COMMON_SENSE_PRIOR:
                use_commensense = true;
                break;
            case srp_md_msgs::GetGoalRequest::NO_FLOAT_PRIOR:
                use_no_float = true;
                break;
        }
    }
    scene_graph.usePriors(use_consistency, use_commensense, use_no_float);

    // Perform inference
    scene_graph.doInference("BP[updates=SEQMAX,maxiter=1000,tol=1e-10,logdomain=0,inference=SUMPROD]", 100, 1e-10);

    // Fill in response
    const std::vector<size_t>& map = scene_graph.getMAP();
    const std::vector<dai::ObjectPair>& pairs = scene_graph.getRelationVars();
    for (size_t i = 0; i < map.size(); ++i)
    {
        dai::ObjectPair pair;
        if (!scene_graph.getRelationVarByLabel(i, &pair))
            continue;
        resp.object1.push_back(pair.object1.name);
        resp.object2.push_back(pair.object2.name);
        resp.relation.push_back(dai::kRelationStrings.at((dai::Relation)map.at(i)));
    }

    scene_graph.visualizeMAP("generated_goal.png");

    return true;
}
