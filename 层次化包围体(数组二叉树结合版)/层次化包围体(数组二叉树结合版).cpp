#include <iostream>
#include <utility>
#include <algorithm>
#include <vector>
#include <memory>
#include <deque>
#include <cassert>
#include "RationalNumber.h"
#include "BVHBinaryTree.h"

struct ArrayBVHTreeNode
{
	bool is_leaf;
	AABB bound_box;
	ArrayBVHTreeNode() = default;
	ArrayBVHTreeNode(const AABB& b, bool i) : bound_box(b), is_leaf(i) {}
	virtual ~ArrayBVHTreeNode() = default;
	virtual void getTriangle() {}
};

struct ArrayBVHTreeNodeLeaf :public ArrayBVHTreeNode
{
	Triangle t;
	ArrayBVHTreeNodeLeaf() = default;
	ArrayBVHTreeNodeLeaf(const Triangle& t, const AABB& a) :ArrayBVHTreeNode(a, true), t(t) {}
	virtual void getTriangle() {}
};

class ArrayBVHTree
{
public:
	ArrayBVHTree() : nodes(), indices() {}
	ArrayBVHTree(BVHTreeNode* cur): nodes(), indices()
	{
		if (cur != nullptr)
		{
			BVHTreeToArrayBVHTree(cur);
		}
	}
	void exchange(ArrayBVHTree& s)
	{
		nodes.swap(s.nodes);
		indices.swap(s.indices);
	}
	const ArrayBVHTree& operator=(const ArrayBVHTree& b)
	{
		if (this != &b)
		{
			ArrayBVHTree temp(b);
			exchange(temp);
		}
		return *this;
	}

	~ArrayBVHTree() = default;
	pair<std::vector<pair<size_t, InterType>>, Value> solveLineAndTriangleInter(PointOrVector3D& p0, PointOrVector3D& d, Value tmin, Value tmax);
	void update(size_t leaf, const Triangle& t);
	const Triangle & getTriangle(size_t leaf) const
	{
		assert(leaf < nodes.size() && nodes[leaf] != nullptr && nodes[leaf]->is_leaf);
		return dynamic_cast<ArrayBVHTreeNodeLeaf*> (nodes[leaf].get())->t;
	}
private:
	pair<std::vector<pair<size_t, InterType>>, Value> searchInter(size_t cur, PointOrVector3D& p0, PointOrVector3D& d, Value tmin, Value tmax, const AABB& current);
	void BVHTreeToArrayBVHTree(BVHTreeNode* cur);
	std::vector<std::shared_ptr<ArrayBVHTreeNode>> nodes;
	std::vector<size_t> indices;  //元素为nodes数组中叶节点的nodes数组索引
};

void ArrayBVHTree::BVHTreeToArrayBVHTree(BVHTreeNode* root)
{
	std::deque<BVHTreeNode*> work_queue;
	work_queue.push_back(root);
	BVHTreeNode** right_end = &work_queue.back();
	bool is_lowest_level = true;

	size_t left_i;
	while (true)
	{
		BVHTreeNode* cur = work_queue.front();
		if (cur == nullptr)
		{
			nodes.emplace_back(nullptr);
			work_queue.push_back(nullptr);
			work_queue.push_back(nullptr);
		}
		else
		{
			if (cur->isleaf == false)
			{
				indices.push_back(nodes.size());
				nodes.emplace_back(new ArrayBVHTreeNode(cur->bound_box, false));
				work_queue.push_back(cur->p.left);
				work_queue.push_back(cur->p.right);
				is_lowest_level = false;
			}
			else
			{
				nodes.emplace_back(new ArrayBVHTreeNodeLeaf(cur->t, cur->bound_box));
				left_i = nodes.size();
				work_queue.push_back(nullptr);
				work_queue.push_back(nullptr);
			}
		}

		if (right_end == &work_queue.front())
		{
			if (is_lowest_level)
			{
				nodes.erase(nodes.begin() + left_i, nodes.end());
				return;
			}
			is_lowest_level = true;
			right_end = &work_queue.back();
		}
		work_queue.pop_front();
	}
}

void ArrayBVHTree::update(size_t leaf, const Triangle& t) //leaf从0开始,注意它为nodes数组的索引
{
	ArrayBVHTreeNodeLeaf* _p = dynamic_cast<ArrayBVHTreeNodeLeaf*> (nodes[leaf].get());
	if (t == _p->t)
		return;

	_p->t = t;
	nodes[leaf]->bound_box.xmin = std::min({ t.v1.x, t.v2.x, t.v3.x });
	nodes[leaf]->bound_box.ymin = std::min({ t.v1.y, t.v2.y, t.v3.y });
	nodes[leaf]->bound_box.zmin = std::min({ t.v1.z, t.v2.z, t.v3.z });
	nodes[leaf]->bound_box.xmax = std::max({ t.v1.x, t.v2.x, t.v3.x });
	nodes[leaf]->bound_box.ymax = std::max({ t.v1.y, t.v2.y, t.v3.y });
	nodes[leaf]->bound_box.zmax = std::max({ t.v1.z, t.v2.z, t.v3.z });

	size_t run = leaf;
	size_t down = leaf;
	AABB _a_b = nodes[down]->bound_box;
	while (run != 0)
	{
		run = (run - 1) / 2;

		AABB& a_b = nodes[run]->bound_box;
		if (2 * run + 1 == down)
		{
			AABB& _right = nodes[2 * run + 2]->bound_box;
			a_b.xmin = std::min(_right.xmin, _a_b.xmin);
			a_b.ymin = std::min(_right.ymin, _a_b.ymin);
			a_b.zmin = std::min(_right.zmin, _a_b.zmin);
			a_b.xmax = std::max(_right.xmax, _a_b.xmax);
			a_b.ymax = std::max(_right.ymax, _a_b.ymax);
			a_b.zmax = std::max(_right.zmax, _a_b.zmax);
		}
		else
		{
			AABB& _left = nodes[2 * run + 1]->bound_box;
			a_b.xmin = std::min(_left.xmin, _a_b.xmin);
			a_b.ymin = std::min(_left.ymin, _a_b.ymin);
			a_b.zmin = std::min(_left.zmin, _a_b.zmin);
			a_b.xmax = std::max(_left.xmax, _a_b.xmax);
			a_b.ymax = std::max(_left.ymax, _a_b.ymax);
			a_b.zmax = std::max(_left.zmax, _a_b.zmax);
		}
		down = run;
		_a_b = a_b;
	}
}

//Value是最近相交点的p+t*d的t值,vector<pair(存放和线段相交的三角形所在BVH树叶节点数组的索引(不经过indices),和三角形的相交类型)>,cur从0开始
pair<std::vector<pair<size_t, InterType>>, Value> ArrayBVHTree::searchInter(size_t cur, PointOrVector3D& p0, PointOrVector3D& d, Value tmin, Value tmax, const AABB& parent)  //要求tmin<=tmax
{
	pair<Value, Value> res;
	AABB _cur = nodes[cur]->bound_box;

	if (!(cur != 0 && parent.xmin == _cur.xmin && parent.ymin == _cur.ymin && parent.zmin == _cur.zmin
		&& parent.xmax == _cur.xmax && parent.ymax == _cur.ymax && parent.zmax == _cur.zmax))
	{
		res = judgeAABBInterline(_cur.xmin, _cur.ymin, _cur.zmin, _cur.xmax, _cur.ymax, _cur.zmax,
			p0.x, p0.y, p0.z, d.x, d.y, d.z, tmin.value, tmax.value, tmin.type, tmax.type);

		if (res.first == v_type::positive_inf)
		{
			return { std::vector<pair<size_t, InterType>>(), Value(v_type::positive_inf) };
		}
	}
	else
	{
		res = { tmin, tmax };
	}

	if (nodes[cur]->is_leaf == false)
	{
		pair<std::vector<pair<size_t, InterType>>, Value> _left = searchInter(2 * cur + 1, p0, d, res.first, res.second, _cur);
		pair<std::vector<pair<size_t, InterType>>, Value> _right = searchInter(2 * cur + 2, p0, d, res.first, res.second, _cur);

		if (_left.second.type == v_type::positive_inf)
		{
			if (_right.second.type == v_type::finite)
			{
				return _right;
			}
			else
			{
				return { std::vector<pair<size_t, InterType>>(), Value(v_type::positive_inf) };
			}
		}
		else
		{
			if (_right.second.type == v_type::finite)
			{
				if (_left.second.value < _right.second.value)
				{
					return _left;
				}
				else if (_left.second.value > _right.second.value)
				{
					return _right;
				}
				else
				{
					_left.first.insert(_left.first.end(), _right.first.begin(), _right.first.end());
					return _left;
				}
			}
			else
			{
				return _left;
			}
		}
	}
	else
	{
		
		Triangle temp = dynamic_cast<ArrayBVHTreeNodeLeaf*> (nodes[cur].get())->t;
		pair<RationalNumber, InterType> _res = lineInterTriangle(p0, d, res.first, res.second, temp.v1, temp.v2, temp.v3);
		if (_res.second != InterType::NOInter)
		{
			std::vector<pair<size_t, InterType>> _r(1);
			_r[0] = { cur, _res.second };
			return { _r, Value(_res.first) };
		}
		else
		{
			return { std::vector<pair<size_t, InterType>>(), Value(v_type::positive_inf) };
		}
	}
}

pair<std::vector<pair<size_t, InterType>>, Value> ArrayBVHTree::solveLineAndTriangleInter(PointOrVector3D& p0, PointOrVector3D& d, Value tmin, Value tmax)
{
	if (nodes.empty())
	{
		return { std::vector<pair<size_t, InterType>>(), Value(v_type::positive_inf) };
	}

	pair<std::vector<pair<size_t, InterType>>, Value> res = searchInter(0, p0, d, tmin, tmax, AABB());
	return res;
}

int main()
{
	using std::vector;
	Triangle t1;
	Triangle t2;
	Triangle t3;
	t1.v1.x = RationalNumber(0, 1);
	t1.v1.y = RationalNumber(0, 1);
	t1.v1.z = RationalNumber(0, 1);
	t1.v2.x = RationalNumber(1, 1);
	t1.v2.y = RationalNumber(0, 1);
	t1.v2.z = RationalNumber(0, 1);
	t1.v3.x = RationalNumber(0, 1);
	t1.v3.y = RationalNumber(1, 1);
	t1.v3.z = RationalNumber(0, 1);

	t2.v1.x = RationalNumber(0, 1);
	t2.v1.y = RationalNumber(0, 1);
	t2.v1.z = RationalNumber(1, 1);
	t2.v2.x = RationalNumber(1, 1);
	t2.v2.y = RationalNumber(0, 1);
	t2.v2.z = RationalNumber(1, 1);
	t2.v3.x = RationalNumber(0, 1);
	t2.v3.y = RationalNumber(1, 1);
	t2.v3.z = RationalNumber(1, 1);

	t3.v1.x = RationalNumber(2, 1);
	t3.v1.y = RationalNumber(0, 1);
	t3.v1.z = RationalNumber(0, 1);
	t3.v2.x = RationalNumber(3, 1);
	t3.v2.y = RationalNumber(0, 1);
	t3.v2.z = RationalNumber(0, 1);
	t3.v3.x = RationalNumber(2, 1);
	t3.v3.y = RationalNumber(1, 1);
	t3.v3.z = RationalNumber(0, 1);

	vector<Triangle> test_tri(3);
	test_tri[0] = t1;
	test_tri[1] = t2;
	test_tri[2] = t3;

	BVHTree tree(test_tri);
	ArrayBVHTree test_tree(tree.getRoot());

	PointOrVector3D _1p;
	_1p.x = RationalNumber(1, 2);
	_1p.y = RationalNumber(1, 2);
	_1p.z = RationalNumber(-1, 1);
	PointOrVector3D _1d;
	_1d.x = RationalNumber(0, 1);
	_1d.y = RationalNumber(0, 1);
	_1d.z = RationalNumber(1, 1);

	pair<std::vector<pair<size_t, InterType>>, Value> r1 = test_tree.solveLineAndTriangleInter(_1p, _1d, Value(r_zero), Value(v_type::positive_inf));
	assert(r1.second != v_type::positive_inf && r1.first.size() == 1 && test_tree.getTriangle(r1.first[0].first) == t1 && r1.second.type == v_type::finite && r1.second.value == RationalNumber(1, 1) && "_1faild!");

	PointOrVector3D _2p;
	_2p.x = RationalNumber(1, 2);
	_2p.y = RationalNumber(1, 2);
	_2p.z = RationalNumber(1, 2);
	PointOrVector3D _2d;
	_2d.x = RationalNumber(0, 1);
	_2d.y = RationalNumber(0, 1);
	_2d.z = RationalNumber(1, 1);

	pair<std::vector<pair<size_t, InterType>>, Value> r2 = test_tree.solveLineAndTriangleInter(_2p, _2d, Value(r_zero), Value(v_type::positive_inf));
	assert(r2.second != v_type::positive_inf && r2.first.size() == 1 && test_tree.getTriangle(r2.first[0].first) == t2 && r2.second.type == v_type::finite && r2.second.value == RationalNumber(1, 2) && "_2faild!");

	PointOrVector3D _3p;
	_3p.x = RationalNumber(5, 2);
	_3p.y = RationalNumber(1, 2);
	_3p.z = RationalNumber(-1, 1);
	PointOrVector3D _3d;
	_3d.x = RationalNumber(0, 1);
	_3d.y = RationalNumber(0, 1);
	_3d.z = RationalNumber(1, 1);

	pair<std::vector<pair<size_t, InterType>>, Value> r3 = test_tree.solveLineAndTriangleInter(_3p, _3d, Value(r_zero), Value(v_type::positive_inf));
	assert(r3.second != v_type::positive_inf && r3.first.size() == 1 && test_tree.getTriangle(r3.first[0].first) == t3 && r3.second.type == v_type::finite && r3.second.value == RationalNumber(1, 1) && "_3faild!");

	PointOrVector3D _4p;
	_4p.x = RationalNumber(1, 2);
	_4p.y = RationalNumber(-1, 1);
	_4p.z = RationalNumber(0, 1);
	PointOrVector3D _4d;
	_4d.x = RationalNumber(0, 1);
	_4d.y = RationalNumber(1, 1);
	_4d.z = RationalNumber(0, 1);

	pair<std::vector<pair<size_t, InterType>>, Value> r4 = test_tree.solveLineAndTriangleInter(_4p, _4d, Value(r_zero), Value(v_type::positive_inf));
	assert(r4.second == v_type::positive_inf && "_4faild!");

	PointOrVector3D _5p;
	_5p.x = RationalNumber(1, 2);
	_5p.y = RationalNumber(1, 2);
	_5p.z = RationalNumber(2, 1);
	PointOrVector3D _5d;
	_5d.x = RationalNumber(0, 1);
	_5d.y = RationalNumber(0, 1);
	_5d.z = RationalNumber(-1, 1);

	pair<std::vector<pair<size_t, InterType>>, Value> r5 = test_tree.solveLineAndTriangleInter(_5p, _5d, Value(r_zero), Value(v_type::positive_inf));
	assert(r5.second != v_type::positive_inf && r5.first.size() == 1 && test_tree.getTriangle(r5.first[0].first) == t2 && r5.second.type == v_type::finite && r5.second.value == RationalNumber(1, 1) && "_5faild!");

	PointOrVector3D _6p;
	_6p.x = RationalNumber(10, 1);
	_6p.y = RationalNumber(10, 1);
	_6p.z = RationalNumber(10, 1);
	PointOrVector3D _6d;
	_6d.x = RationalNumber(-1, 1);
	_6d.y = RationalNumber(-1, 1);
	_6d.z = RationalNumber(-1, 1);

	pair<std::vector<pair<size_t, InterType>>, Value> r6 = test_tree.solveLineAndTriangleInter(_6p, _6d, Value(r_zero), Value(v_type::positive_inf));
	assert(r6.second != v_type::positive_inf && r6.first.size() == 1 && test_tree.getTriangle(r6.first[0].first) == t1 && r6.second.type == v_type::finite && r6.second.value == RationalNumber(10, 1) && "_6faild!");

	PointOrVector3D _7p;
	_7p.x = RationalNumber(1, 2);
	_7p.y = RationalNumber(1, 2);
	_7p.z = RationalNumber(1, 2);
	PointOrVector3D _7d;
	_7d.x = RationalNumber(1, 1);
	_7d.y = RationalNumber(0, 1);
	_7d.z = RationalNumber(0, 1);

	pair<std::vector<pair<size_t, InterType>>, Value> r7 = test_tree.solveLineAndTriangleInter(_7p, _7d, Value(r_zero), Value(v_type::positive_inf));
	assert(r7.second == v_type::positive_inf && "_7faild!");

	PointOrVector3D _8p;
	_8p.x = RationalNumber(4, 5);
	_8p.y = RationalNumber(4, 5);
	_8p.z = RationalNumber(-1, 1);
	PointOrVector3D _8d;
	_8d.x = RationalNumber(0, 1);
	_8d.y = RationalNumber(0, 1);
	_8d.z = RationalNumber(1, 1);

	pair<std::vector<pair<size_t, InterType>>, Value> r8 = test_tree.solveLineAndTriangleInter(_8p, _8d, Value(r_zero), Value(v_type::positive_inf));
	assert(r8.second == v_type::positive_inf && "_8faild!");

	PointOrVector3D _9p;
	_9p.x = RationalNumber(4, 5);
	_9p.y = RationalNumber(4, 5);
	_9p.z = RationalNumber(1, 2);
	PointOrVector3D _9d;
	_9d.x = RationalNumber(0, 1);
	_9d.y = RationalNumber(0, 1);
	_9d.z = RationalNumber(1, 1);

	pair<std::vector<pair<size_t, InterType>>, Value> r9 = test_tree.solveLineAndTriangleInter(_9p, _9d, Value(r_zero), Value(v_type::positive_inf));
	assert(r9.second == v_type::positive_inf && "_9faild!");
	return 0;
}



