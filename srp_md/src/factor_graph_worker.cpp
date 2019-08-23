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
    // Make a vector of new objects
    std::vector<dai::Object> objs;
    objs.reserve(req.objects.size());
    for (size_t i = 0; i < req.objects.size(); ++i)
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
        objs.emplace_back(req.objects[i], cls, i, req.num_states[i]);
    }

    // Build scene graph from objects
    // dai::sceneGraph scene_graph(objs);

    // Perform inference
    // scene_graph.doInference("BP[updates=SEQMAX,maxiter=10000,tol=1e-10,logdomain=0,inference=SUMPROD]", 1, 1e-10);

    // Fill in response
    // const std::vector<size_t>& map = scene_graph.getMAP();
    // const std::vector<dai::ObjectPair>& pairs = scene_graph.getAllRelations();
    // for (size_t i = 0; i < map.size(); ++i)
    // {
    //     const dai::ObjectPair& pair = pairs[i];

    //     resp.object1.push_back(pair.object1.name);
    //     resp.object2.push_back(pair.object2.name);
    //     resp.relation.push_back(scene_graph._relation_strs[map[i]]);
    // }

    // TODO(Kevin): Delete this
    // Temporary dummy data for testing
    for (size_t i = 0; i < req.objects.size(); ++i)
    {
        for (size_t j = i + 1; j < req.objects.size(); ++j)
        {
            resp.object1.push_back(req.objects[i]);
            resp.object2.push_back(req.objects[j]);
            resp.relation.push_back(dai::sceneGraph::kRelationStrings.at(rand() % dai::Relation::kNumRelations));
        }
    }

    return true;
}
