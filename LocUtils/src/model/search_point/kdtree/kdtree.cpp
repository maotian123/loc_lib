#include "LocUtils/model/search_point/kdtree/kdtree.h"

#include <glog/logging.h>

namespace LocUtils
{
    /***************************************************************************
    algrithon kdtree
    ***************************************************************************/
    bool KdTree::BuildTree(const CloudPtr &input_cloud)
    {
        if(input_cloud->empty())
        {
            return false;
        }
        
        cloud_.clear();
        cloud_ = CloudToVec3fBuff(*input_cloud);

        Clear();
        Reset();
        
        const int cloud_size = cloud_.size();
        IndexVec idx(cloud_size);
        for (int i = 0; i < cloud_size; ++i) {
            idx[i] = i;
        }

        Insert(idx, root_.get());

    }

    /// 清理数据
    void KdTree::Clear()
    {
        //防止内存泄露
        for (const auto &np : nodes_) 
        {
            if (np.second != root_.get()) {
                delete np.second;
            }
        }

        nodes_.clear();
        root_ = nullptr;
        size_ = 0;
        tree_node_id_ = 0;
    }

    void KdTree::Reset()
    {
        tree_node_id_ = 0;
        root_.reset(new KdTreeNode());
        root_->id_ = tree_node_id_++;
        size_ = 0;
    }

    void KdTree::Insert(const IndexVec& points, KdTreeNode* node)
    {
        nodes_.insert({node->id_, node});

        if(points.empty())
        {
            return;
        }

        if(points.size() == 1)
        {
            size_++;
            node->point_idx_ = points[0];
            return;
        }

        IndexVec left, right;
        
        if (!FindSplitAxisAndThresh(points, node->axis_index_, node->split_thresh_, left, right)) 
        {
            size_++;
            node->point_idx_ = points[0];
            return;
        }

        const auto create_if_not_empty = [&node, this](KdTreeNode *&new_node, const IndexVec &index) 
        {
            if (!index.empty()) {
                new_node = new KdTreeNode;
                new_node->id_ = tree_node_id_++;
                Insert(index, new_node);
            }
        };

        create_if_not_empty(node->left_, left);
        create_if_not_empty(node->right_, right);
    }

    bool KdTree::FindSplitAxisAndThresh(const IndexVec &point_idx, int &axis, float &th, IndexVec &left, IndexVec &right) 
    {
        // 计算三个轴上的散布情况，我们使用math_utils.h里的函数
        Vec3f var;
        Vec3f mean;
        math::ComputeMeanAndCovDiag(point_idx, mean, var, [this](int idx) { return cloud_[idx]; });
        int max_i, max_j;
        var.maxCoeff(&max_i, &max_j);
        axis = max_i;
        th = mean[axis];

        for (const auto &idx : point_idx) {
            if (cloud_[idx][axis] < th) {
                // 中位数可能向左取整
                left.emplace_back(idx);
            } else {
                right.emplace_back(idx);
            }
        }

        // 边界情况检查：输入的points等于同一个值，上面的判定是>=号，所以都进了右侧
        // 这种情况不需要继续展开，直接将当前节点设为叶子就行
        if (point_idx.size() > 1 && (left.empty() || right.empty())) {
            return false;
        }

        return true;
    }

    bool KdTree::GetClosestPoint(const PointType& pt, std::vector<int>& closest_idx, int k)
    {
        if (k > size_) 
        {
            LOG(ERROR) << "cannot set k larger than cloud size: " << k << ", " << size_;
            return false;
        }
        k_ = k;

        std::priority_queue<NodeAndDistance> knn_result;

        Knn(ToVec3f(pt), root_.get(), knn_result);
        // 排序并返回结果
        closest_idx.resize(knn_result.size());
        for (int i = closest_idx.size() - 1; i >= 0; --i) {
            // 倒序插入
            closest_idx[i] = knn_result.top().node_->point_idx_;
            knn_result.pop();
        }
        return true;
    }

    bool KdTree::GetClosestPoint(const Vec3f& pt, std::vector<int>& closest_idx, int k)
    {
        if (k > size_) 
        {
            LOG(ERROR) << "cannot set k larger than cloud size: " << k << ", " << size_;
            return false;
        }
        k_ = k;

        std::priority_queue<NodeAndDistance> knn_result;

        Knn(pt, root_.get(), knn_result);
        // 排序并返回结果
        closest_idx.resize(knn_result.size());
        for (int i = closest_idx.size() - 1; i >= 0; --i) {
            // 倒序插入
            closest_idx[i] = knn_result.top().node_->point_idx_;
            knn_result.pop();
        }
        return true;
    }

    void KdTree::Knn(const Vec3f& pt, KdTreeNode* node, std::priority_queue<NodeAndDistance>& knn_result) const
    {
        if (node->IsLeaf()) 
        {
            // 如果是叶子，检查叶子是否能插入
            ComputeDisForLeaf(pt, node, knn_result);
            return;
        }
        // 看pt落在左还是右，优先搜索pt所在的子树
        // 然后再看另一侧子树是否需要搜索
        KdTreeNode *this_side, *that_side;
        if (pt[node->axis_index_] < node->split_thresh_) 
        {
            this_side = node->left_;
            that_side = node->right_;
        }
        else 
        {
            this_side = node->right_;
            that_side = node->left_;
        }

        Knn(pt, this_side, knn_result);
        if (NeedExpand(pt, node, knn_result)) {  // 注意这里是跟自己比
            Knn(pt, that_side, knn_result);
        }
    }

    void KdTree::ComputeDisForLeaf(const Vec3f &pt, KdTreeNode *node,
                               std::priority_queue<NodeAndDistance> &knn_result) const 
    {
        // 比较与结果队列的差异，如果优于最远距离，则插入
        float dis2 = Dis2(pt, cloud_[node->point_idx_]);
        if (knn_result.size() < k_) {
            // results 不足k
            knn_result.emplace(node, dis2);
        } else {
            // results等于k，比较current与max_dis_iter之间的差异
            if (dis2 < knn_result.top().distance2_) {
                knn_result.emplace(node, dis2);
                knn_result.pop();
            }
        }
    }

    bool KdTree::NeedExpand(const Vec3f &pt, KdTreeNode *node, std::priority_queue<NodeAndDistance> &knn_result) const 
    {
        if (knn_result.size() < k_) {
            return true;
        }

        if (approximate_) {
            float d = pt[node->axis_index_] - node->split_thresh_;
            if ((d * d) < knn_result.top().distance2_ * alpha_) {
                return true;
            } else {
                return false;
            }
        } else {
            // 检测切面距离，看是否有比现在更小的
            float d = pt[node->axis_index_] - node->split_thresh_;
            if ((d * d) < knn_result.top().distance2_) {
                return true;
            } else {
                return false;
            }
        }
    }
    
    void KdTree::PrintAll() 
    {
        for (const auto &np : nodes_) {
            auto node = np.second;
            if (node->left_ == nullptr && node->right_ == nullptr) {
                LOG(INFO) << "leaf node: " << node->id_ << ", idx: " << node->point_idx_;
            } else {
                LOG(INFO) << "node: " << node->id_ << ", axis: " << node->axis_index_ << ", th: " << node->split_thresh_;
            }
        }
    }
    /***************************************************************************
    registration
    ***************************************************************************/
    KdtreeRegistration::KdtreeRegistration(bool use_multi):
        b_use_m_(use_multi)
        ,source_cloud_(new PointCloudType)      
    {
        kdtree_ptr_ = std::make_shared<KdTree>();

        LOG(INFO) << "CREATE KDTREE SEARCH ";
    }

    bool KdtreeRegistration::SetTargetCloud(const CloudPtr &cloud)
    {
        if(cloud->points.empty())
        {
            return false;
        }
        *source_cloud_ = *cloud;
        kdtree_ptr_->BuildTree(source_cloud_);
        return true;
    }

    std::vector<int> KdtreeRegistration::FindNearstPoints(const Vec3f& point, int k)
    {
        std::vector<int> result;
        kdtree_ptr_->GetClosestPoint(point, result, k);
        // 
        float max_dis = math::PointDis(ToVec3f(source_cloud_->points[result.back()]), point);
        float min_dis = math::PointDis(ToVec3f(source_cloud_->points[result.front()]), point);

        // LOG(INFO) << "kdtree search max dis " << max_dis;
        // LOG(INFO) << "kdtree search min dis " << min_dis;
        return result;
    }

    void KdtreeRegistration::SetEnableANN(bool use_ann, float alpha)
    {
        kdtree_ptr_->SetEnableANN(use_ann, alpha);
    }

    void KdtreeRegistration::FindCloud(const CloudPtr &cloud2, std::vector<std::pair<size_t, size_t>>& matches)
    {

    }

}