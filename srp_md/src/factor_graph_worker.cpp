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
        dai::VarSet vars;
        // Get the Vars for singular objects
        for (const auto& obj_name : ros_factor.objects)
        {
            vars.insert(scene_graph.getObjectVarByName(obj_name));
        }
        // Get the Vars for pairs of objects
        for (const auto& pair : ros_factor.pairs)
        {
            vars.insert(scene_graph.getRelationVarByNames(pair.object1, pair.object2));
        }
        // TODO(Kevin): Make sure probs are ordered correctly
        // learned_factors.emplace_back(vars, ros_factor.probs);
        scene_graph.addFactor(vars, ros_factor.probs);
    }

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
