#include <srp_md/factor_graph_worker.h>

FactorGraphWorker::FactorGraphWorker()
{
}

void FactorGraphWorker::Setup()
{
    ros::NodeHandle nh;
    goal_server_ = nh.advertiseService("get_goal", &FactorGraphWorker::GetGoal, this);
}

bool FactorGraphWorker::GetGoal(srp_md::GetGoalRequest& req, srp_md::GetGoalResponse& resp)
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
            case srp_md::GetGoalRequest::CLASS_CONTAINER:
                cls = dai::ObjectClass::kContainer;
                break;
            case srp_md::GetGoalRequest::CLASS_SUPPORTER:
                cls = dai::ObjectClass::kSupporter;
                break;
            case srp_md::GetGoalRequest::CLASS_PROP:
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
            dai::Var v = scene_graph.getRelationVarByNames(pair.object1, pair.object2);
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
    for (auto& prior : req.prior_knowledge)
    {
        switch (prior)
        {
            case srp_md::GetGoalRequest::CONSISTENCY_PRIOR:
                use_consistency = true;
                break;
            case srp_md::GetGoalRequest::COMMON_SENSE_PRIOR:
                use_commensense = true;
                break;
        }
    }
    scene_graph.usePriors(use_consistency, use_commensense);

    // Perform inference
    scene_graph.doInference("BP[updates=SEQMAX,maxiter=10000,tol=1e-10,logdomain=0,inference=SUMPROD]", 1, 1e-10);

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
        resp.relation.push_back(dai::sceneGraph::kRelationStrings.at(map.at(i)));
    }

    return true;
}
