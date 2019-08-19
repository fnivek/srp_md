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
        objs.emplace_back(req.objects[i].data, dai::ObjectClass::Prop, i);
    }

    // Build scene graph from objects
    dai::sceneGraph scene_graph(objs);

    // Perform inference
    scene_graph.doInference("BP[updates=SEQMAX,maxiter=10000,tol=1e-10,logdomain=0,inference=SUMPROD]", 1, 1e-10);

    // Fill in response
    const std::vector<size_t>& map = scene_graph.getMAP();
    const std::vector<dai::ObjectPair>& pairs = scene_graph.getAllRelations();
    for(size_t i = 0; i < map.size(); ++i)
    {
        const dai::ObjectPair& pair = pairs[i];

        std_msgs::String str;
        str.data = pair.object1.name;
        resp.object1.push_back(str);

        str.data = pair.object2.name;
        resp.object2.push_back(str);

        str.data = scene_graph._relation_strs[map[i]];
        resp.relation.push_back(str);
    }

    return true;
}
