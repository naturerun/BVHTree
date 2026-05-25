#include <iostream>
#include <utility>
#include <algorithm>
#include <vector>
#include <cassert>
#include "RationalNumber.h"

using std::pair;

RationalNumber r_zero(0, 1);

enum class v_type { positive_inf, negative_inf, finite };

struct Value
{
	RationalNumber value;
	v_type type;
	Value() = default;
	Value(RationalNumber _v) :value(_v), type(v_type::finite) {}
	Value(v_type v_t) :type(v_t), value(0, 1) {}
	bool operator<(const Value& be) const
	{
		if (type == be.type)
		{
			if (type == v_type::finite)
			{
				return value < be.value;
			}
			return false;
		}
		return type == v_type::negative_inf && (be.type == v_type::finite || be.type == v_type::positive_inf)
			|| type == v_type::finite && be.type == v_type::positive_inf;
	}

	Value operator-()
	{
		if (type == v_type::negative_inf)
		{
			return Value(v_type::positive_inf);
		}

		if (type == v_type::positive_inf)
		{
			return Value(v_type::negative_inf);
		}

		return Value(-value);
	}

	Value operator-(Value& be)
	{
		return (*this) + (-be);
	}

	Value operator+(const Value& be)
	{
		Value zero{ r_zero };
		if (type == v_type::negative_inf)
		{
			if (be.type == v_type::positive_inf)
			{
				return zero;
			}
			return Value(v_type::negative_inf);
		}

		else if (type == v_type::positive_inf)
		{
			if (be.type == v_type::negative_inf)
			{
				return zero;
			}
			return Value(v_type::positive_inf);
		}
		else
		{
			if (be.type == v_type::negative_inf)
			{
				return Value(v_type::negative_inf);
			}
			else if (be.type == v_type::positive_inf)
			{
				return Value(v_type::positive_inf);
			}
			else
			{
				return Value(value + be.value);
			}
		}
	}

	Value operator*(const Value& be)
	{
		const Value zero{ r_zero };
		if (be == zero || (*this) == zero)
			return zero;

		if (be.type != type)
		{
			if (type == v_type::negative_inf)
			{
				if (be.type == v_type::finite)
				{
					if (be > zero)
					{
						return Value(v_type::negative_inf);

					}
					return Value(v_type::positive_inf);
				}
				return Value(v_type::negative_inf);
			}

			if (type == v_type::positive_inf)
			{
				if (be.type == v_type::finite)
				{
					if (be > zero)
					{
						return Value(v_type::positive_inf);

					}
					return Value(v_type::negative_inf);
				}
				return Value(v_type::negative_inf);
			}

			if ((*this) > zero)
			{
				if (be.type == v_type::positive_inf)
				{
					return Value(v_type::positive_inf);
				}
				return Value(v_type::negative_inf);
			}
			else
			{
				if (be.type == v_type::positive_inf)
				{
					return Value(v_type::negative_inf);
				}
				return Value(v_type::positive_inf);
			}
		}

		if (type == v_type::finite)
		{
			return Value(value * be.value);
		}
		return Value(v_type::positive_inf);
	}

	bool operator==(const Value& be) const
	{
		if (type == be.type)
		{
			if (type == v_type::finite)
			{
				return value == be.value;
			}
			return true;
		}
		return false;
	}

	bool operator!=(const Value& be) const
	{
		return !operator == (be);
	}

	bool operator<=(const Value& be) const
	{
		return !operator>(be);
	}

	bool operator>=(const Value& be) const
	{
		return !operator<(be);
	}

	bool operator>(const Value& be) const
	{
		return operator <(be) == false && operator==(be) == false;
	}
};

Value zero{ r_zero };
pair<Value, Value> getIntersection(pair<Value, Value> l, pair<Value, Value> j, bool& inter) //返回区间l,j的交区间,l,g左右端点均为正整数,返回(0,0)表示交集不存在
{
	if (l.second < j.first || l.first > j.second)
	{
		inter = false;
		return { zero , zero };
	}

	inter = true;
	if (j.first <= l.first)
	{
		if (j.second < l.second)
			return { l.first, j.second };
		return l;
	}

	if (j.second <= l.second)
		return j;
	return { j.first, l.second };
}

RationalNumber compute_t(RationalNumber c, RationalNumber C, RationalNumber value, bool& inf)
{
	if (C == r_zero)
	{
		inf = true;
		return r_zero;
	}

	inf = false;
	return (value - c) / C;
}
//(xmin, ymin, zmin)是AABB下侧矩形左下角的顶点,(xmax, ymax, zmax)是AABB上侧矩形右上角的顶点,线段为(x0,y0,z0)+t*(A,B,C),t在tmin,tmax之间(包括tmin,tmax),tmin可以等于小于大于tmax
//当返回值的first成员为v_type::positive_inf时,线段和AABB不相交,否则为线段和AABB相交部分的参数t变化范围[first,second],返回的pair满足first<=second,first和second均为有限值
pair<Value, Value> judgeAABBInterline(RationalNumber xmin, RationalNumber ymin, RationalNumber zmin, RationalNumber xmax, RationalNumber ymax, RationalNumber zmax,
	RationalNumber x0, RationalNumber y0, RationalNumber z0, RationalNumber A, RationalNumber B, RationalNumber C, RationalNumber tmin, RationalNumber tmax,
	v_type tmin_type, v_type tmax_type)
{
	Value _x0{ x0 };
	Value _y0{ y0 };
	Value _z0{ z0 };
	Value _A{ A };
	Value _B{ B };
	Value _C{ C };
	Value _xmin{ xmin };
	Value _xmax{ xmax };
	Value _ymin{ ymin };
	Value _ymax{ ymax };
	Value _zmin{ zmin };
	Value _zmax{ zmax };

	Value _tmin;
	if (tmin_type == v_type::finite)
	{
		_tmin = Value{ tmin };
	}
	else
	{
		_tmin = Value{ tmin_type };
	}

	Value _tmax;
	if (tmax_type == v_type::finite)
	{
		_tmax = Value{ tmax };
	}
	else
	{
		_tmax = Value{ tmax_type };
	}

	Value x_range_left = (_x0 + _A * _tmin);
	Value x_range_right = (_x0 + _A * _tmax);
	Value y_range_left = (_y0 + _B * _tmin);
	Value y_range_right = (_y0 + _B * _tmax);
	Value z_range_left = (_z0 + _C * _tmin);
	Value z_range_right = (_z0 + _C * _tmax);

	bool x_exchange = false;
	if (x_range_left > x_range_right)
	{
		std::swap(x_range_left, x_range_right);
		x_exchange = true;
	}

	bool y_exchange = false;
	if (y_range_left > y_range_right)
	{
		std::swap(y_range_left, y_range_right);
		y_exchange = true;
	}

	bool z_exchange = false;
	if (z_range_left > z_range_right)
	{
		std::swap(z_range_left, z_range_right);
		z_exchange = true;
	}

	bool x_is_inter;
	pair<Value, Value> x_inter = getIntersection({ x_range_left, x_range_right }, { _xmin, _xmax }, x_is_inter);
	if (x_is_inter == false)
		return { Value{v_type::positive_inf}, Value{v_type::positive_inf} };

	bool y_is_inter;
	pair<Value, Value> y_inter = getIntersection({ y_range_left, y_range_right }, { _ymin, _ymax }, y_is_inter);
	if (y_is_inter == false)
		return { Value{v_type::positive_inf}, Value{v_type::positive_inf} };

	bool z_is_inter;
	pair<Value, Value> z_inter = getIntersection({ z_range_left, z_range_right }, { _zmin, _zmax }, z_is_inter);
	if (z_is_inter == false)
		return { Value{v_type::positive_inf}, Value{v_type::positive_inf} };

	Value t_x_min;
	if (x_inter.first != x_range_left)
	{
		bool inf;
		t_x_min = Value(compute_t(_x0.value, _A.value, x_inter.first.value, inf));
	}
	else
	{
		t_x_min = _tmin;
	}

	Value t_x_max;
	if (x_inter.second != x_range_right)
	{
		bool inf;
		t_x_max = Value(compute_t(_x0.value, _A.value, x_inter.second.value, inf));
	}
	else
	{
		t_x_max = _tmax;
	}

	if (t_x_min > t_x_max)
	{
		std::swap(t_x_min, t_x_max);
	}


	Value t_y_min;
	if (y_inter.first != y_range_left)
	{
		bool inf;
		t_y_min = Value(compute_t(_y0.value, _B.value, y_inter.first.value, inf));
	}
	else
	{
		t_y_min = _tmin;
	}

	Value t_y_max;
	if (y_inter.second != y_range_right)
	{
		bool inf;
		t_y_max = Value(compute_t(_y0.value, _B.value, y_inter.second.value, inf));
	}
	else
	{
		t_y_max = _tmax;
	}

	if (t_y_min > t_y_max)
	{
		std::swap(t_y_min, t_y_max);
	}

	Value t_z_min;
	if (z_inter.first != z_range_left)
	{
		bool inf;
		t_z_min = Value(compute_t(_z0.value, _C.value, z_inter.first.value, inf));
	}
	else
	{
		t_z_min = _tmin;
	}

	Value t_z_max;
	if (z_inter.second != z_range_right)
	{
		bool inf;
		t_z_max = Value(compute_t(_z0.value, _C.value, z_inter.second.value, inf));
	}
	else
	{
		t_z_max = _tmax;
	}

	if (t_z_min > t_z_max)
	{
		std::swap(t_z_min, t_z_max);
	}

	bool inter_xy;
	pair<Value, Value> xy = getIntersection({ t_x_min, t_x_max }, { t_y_min, t_y_max }, inter_xy);
	if (inter_xy == false)
		return { Value{v_type::positive_inf}, Value{v_type::positive_inf} };

	bool inter_xyz;
	pair<Value, Value> xyz = getIntersection(xy, { t_z_min, t_z_max }, inter_xyz);
	if (inter_xyz == false)
		return { Value{v_type::positive_inf}, Value{v_type::positive_inf} };
	return xyz;
}

struct PointOrVector3D
{
	RationalNumber x;
	RationalNumber y;
	RationalNumber z;
	bool operator==(const PointOrVector3D& be) const
	{
		return x == be.x && y == be.y && z == be.z;
	}
	PointOrVector3D() = default;
};

RationalNumber innerProduct(PointOrVector3D& v1, PointOrVector3D& v2)
{
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

PointOrVector3D crossProduct(PointOrVector3D& v1, PointOrVector3D& v2)
{
	PointOrVector3D res;
	res.x = v1.y * v2.z - v1.z * v2.y;
	res.y = v1.z * v2.x - v1.x * v2.z;
	res.z = v1.x * v2.y - v1.y * v2.x;
	return res;
}

PointOrVector3D add(PointOrVector3D& v1, PointOrVector3D& v2)
{
	PointOrVector3D res;
	res.x = v1.x + v2.x;
	res.y = v1.y + v2.y;
	res.z = v1.z + v2.z;
	return res;
}

PointOrVector3D subtract(PointOrVector3D& v1, PointOrVector3D& v2)
{
	PointOrVector3D res;
	res.x = v1.x - v2.x;
	res.y = v1.y - v2.y;
	res.z = v1.z - v2.z;
	return res;
}

enum class InterType { V1, V3, V2, V1V3, V3V2, V2V1, Internal, NOInter }; //三角形从V1出发逆时针依次为v1,v3,v2
pair<RationalNumber, InterType> lineInterTriangle(PointOrVector3D& p0, PointOrVector3D& d, Value tmin, Value tmax, PointOrVector3D& v1, PointOrVector3D& v2, PointOrVector3D& v3)
{
	PointOrVector3D v1v2 = subtract(v1, v2);
	PointOrVector3D v1v3 = subtract(v1, v3);
	PointOrVector3D n = crossProduct(v1v3, v1v2);
	if (innerProduct(n, d) == r_zero)  //线段在平面内或和平面平形均视为光线未击中平面
	{
		return { r_zero, InterType::NOInter };
	}

	PointOrVector3D s = subtract(v1, p0);
	RationalNumber l = innerProduct(n, s) / innerProduct(n, d);
	Value _l(l);

	if (tmin > tmax)
	{
		std::swap(tmin, tmax);
	}

	if (tmin > _l || _l > tmax)
	{
		return { r_zero, InterType::NOInter };
	}

	PointOrVector3D res;
	res.x = p0.x + d.x * l;
	res.y = p0.y + d.y * l;
	res.z = p0.z + d.z * l;

	PointOrVector3D v_v3v1 = subtract(v3, v1);
	PointOrVector3D v_v2v3 = subtract(v2, v3);
	PointOrVector3D v_v1v2 = subtract(v1, v2);
	PointOrVector3D v_rv1 = subtract(res, v1);
	PointOrVector3D v_rv3 = subtract(res, v3);
	PointOrVector3D v_rv2 = subtract(res, v2);
	PointOrVector3D c_1 = crossProduct(v_v3v1, v_rv1);
	PointOrVector3D c_2 = crossProduct(v_v2v3, v_rv3);
	PointOrVector3D c_3 = crossProduct(v_v1v2, v_rv2);

	RationalNumber _c_1;
	RationalNumber _c_2;
	RationalNumber _c_3;
	if ((_c_1 = innerProduct(c_1, n)) >= r_zero && (_c_2 = innerProduct(c_2, n)) >= r_zero && (_c_3 = innerProduct(c_3, n)) >= r_zero)
	{
		if (_c_1 == r_zero)
		{
			if (res.x == v1.x && res.y == v1.y && res.z == v1.z)
			{
				return { l, InterType::V1 };
			}
			else if (res.x == v3.x && res.y == v3.y && res.z == v3.z)
			{
				return { l, InterType::V3 };
			}
			else
			{
				return { l, InterType::V1V3 };
			}
		}

		if (_c_2 == r_zero)
		{
			if (res.x == v2.x && res.y == v2.y && res.z == v2.z)
			{
				return { l, InterType::V2 };
			}
			else
			{
				return { l, InterType::V3V2 };
			}
		}

		if (_c_3 == r_zero)
		{
			return { l, InterType::V2V1 };
		}

		return { l, InterType::Internal };
	}
	return { r_zero, InterType::NOInter };
}

struct Triangle
{
	PointOrVector3D v1;
	PointOrVector3D v2;
	PointOrVector3D v3;
	bool operator==(const Triangle &be) const
	{
		return v1 == be.v1 && v2 == be.v2 && v3 == be.v3;
	}
};

struct AABB
{
	RationalNumber xmin;
	RationalNumber ymin;
	RationalNumber zmin;
	RationalNumber xmax;
	RationalNumber ymax;
	RationalNumber zmax;
};

struct BVHTreeNode
{
	AABB bound_box;
	BVHTreeNode() = default;
	BVHTreeNode(const AABB& b) : bound_box(b) {}
};

struct BVHTreeNodeLeaf :public BVHTreeNode
{
	Triangle t;
	BVHTreeNodeLeaf() = default;
	BVHTreeNodeLeaf(const Triangle& t, const AABB& a) :BVHTreeNode(a), t(t) {}
};

struct Compare
{
	bool operator()(size_t left, size_t right) const
	{
		return mid_array[left] < mid_array[right];
	}
	Compare(std::vector<RationalNumber>& m) :mid_array(m) {}
	std::vector<RationalNumber>& mid_array;
};

class BVHTree
{
public:
	BVHTree() : interval_nodes(), leaf_nodes(), indices(), leaf_nodes_index_to_indices_index(){}
	BVHTree(const std::vector<Triangle>& _t);
	BVHTree(const BVHTree& b) = default;
	void exchange(BVHTree& s)
	{ 
		std::swap(low_ext, s.low_ext); 
	    std::swap(left_down_num, s.left_down_num); 
	    std::swap(right_down_num, s.right_down_num);
		interval_nodes.swap(s.interval_nodes);
		leaf_nodes.swap(s.leaf_nodes);
		indices.swap(s.indices);
		leaf_nodes_index_to_indices_index.swap(s.leaf_nodes_index_to_indices_index);
	}
	const BVHTree& operator=(const BVHTree& b)
	{
		if (this != &b)
		{
			BVHTree temp(b);
			exchange(temp);
		}
		return *this;
	}

	~BVHTree() = default;
	pair<std::vector<pair<size_t, InterType>>, Value> solveLineAndTriangleInter(PointOrVector3D& p0, PointOrVector3D& d, Value tmin, Value tmax);
	void update(size_t leaf, const Triangle& t);    //leaf从0开始,注意它为leaf_nodes数组的索引而不是indices数组的索引
	const BVHTreeNodeLeaf& getLeaf(size_t leaf_node_index) { return leaf_nodes[indices[node_index_to_array_index(leaf_node_index)]]; } 
private:
	void CreateBVHTree(size_t cur);
	pair<std::vector<pair<size_t, InterType>>, Value> searchInter(size_t cur, PointOrVector3D& p0, PointOrVector3D& d, Value tmin, Value tmax, const AABB& current);
	void adjustIndices(std::vector<RationalNumber>& mid_array_x,
		std::vector<RationalNumber>& mid_array_y, std::vector<RationalNumber>& mid_array_z, size_t left, size_t right);
	size_t array_index_to_node_index(size_t array_index);
	size_t node_index_to_array_index(size_t array_index);
	AABB& getAABB(size_t cur);
	std::vector<BVHTreeNode> interval_nodes;
	std::vector<BVHTreeNodeLeaf> leaf_nodes;
	std::vector<size_t> indices;
	std::vector<size_t> leaf_nodes_index_to_indices_index;
	size_t low_ext = 0;
	size_t left_down_num = 0;
	size_t right_down_num = 0;
};

AABB& BVHTree::getAABB(size_t cur)
{
	if (cur < leaf_nodes.size() - 1)
	{
		return interval_nodes[cur].bound_box;
	}
	else
	{
		return leaf_nodes[indices[node_index_to_array_index(cur)]].bound_box;
	}
}

size_t BVHTree::node_index_to_array_index(size_t node_index) //node_index从0开始
{
	++node_index;
	if (node_index - (leaf_nodes.size() - 1) <= right_down_num)
	{
		return left_down_num + node_index - (leaf_nodes.size() - 1) - 1;
	}
	else
	{
		return node_index - (leaf_nodes.size() - 1) - right_down_num - 1;
	}
}

size_t BVHTree::array_index_to_node_index(size_t array_index) //array_index从0开始
{
	++array_index;
	if (array_index <= left_down_num)
	{
		return leaf_nodes.size() - 1 + right_down_num + array_index - 1;
	}
	else
	{
		return array_index - left_down_num + leaf_nodes.size() - 1 - 1;
	}
}

void BVHTree::adjustIndices(std::vector<RationalNumber>& mid_array_x,
	std::vector<RationalNumber>& mid_array_y, std::vector<RationalNumber>& mid_array_z, size_t left, size_t right)
{
	if (right == left)
		return;
	RationalNumber x_minest = leaf_nodes[indices[left]].bound_box.xmin;
	RationalNumber x_maxest = leaf_nodes[indices[left]].bound_box.xmax;
	RationalNumber y_minest = leaf_nodes[indices[left]].bound_box.ymin;
	RationalNumber y_maxest = leaf_nodes[indices[left]].bound_box.ymax;
	RationalNumber z_minest = leaf_nodes[indices[left]].bound_box.zmin;
	RationalNumber z_maxest = leaf_nodes[indices[left]].bound_box.zmax;
	for (size_t i = left + 1; i < right; ++i)
	{
		x_minest = std::min(x_minest, leaf_nodes[indices[i]].bound_box.xmin);
		x_maxest = std::max(x_maxest, leaf_nodes[indices[i]].bound_box.xmax);
		y_minest = std::min(y_minest, leaf_nodes[indices[i]].bound_box.ymin);
		y_maxest = std::max(y_maxest, leaf_nodes[indices[i]].bound_box.ymax);
		z_minest = std::min(z_minest, leaf_nodes[indices[i]].bound_box.zmin);
		z_maxest = std::max(z_maxest, leaf_nodes[indices[i]].bound_box.zmax);
	}

	RationalNumber max_length = std::max({ x_maxest - x_minest, y_maxest - y_minest, z_maxest - z_minest });

	if (max_length == x_maxest - x_minest)
	{
		nth_element(indices.begin() + left, indices.begin() + (left + right - 1) / 2, indices.begin() + right, Compare(mid_array_x));
	}
	else if (max_length == y_maxest - y_minest)
	{
		nth_element(indices.begin() + left, indices.begin() + (left + right - 1) / 2, indices.begin() + right, Compare(mid_array_y));
	}
	else
	{
		nth_element(indices.begin() + left, indices.begin() + (left + right - 1) / 2, indices.begin() + right, Compare(mid_array_z));
	}

	size_t mid = (left + right - 1) / 2;
	adjustIndices(mid_array_x, mid_array_y, mid_array_z, left, mid);
	adjustIndices(mid_array_x, mid_array_y, mid_array_z, mid + 1, right);
}

void BVHTree::update(size_t leaf, const Triangle& t) //leaf从0开始,注意它为leaf_nodes数组的索引而不是indices数组的索引
{
	if (t == leaf_nodes[leaf].t)
		return;

	leaf_nodes[leaf].t = t;
	leaf_nodes[leaf].bound_box.xmin = std::min({ t.v1.x, t.v2.x, t.v3.x });
	leaf_nodes[leaf].bound_box.ymin = std::min({ t.v1.y, t.v2.y, t.v3.y });
	leaf_nodes[leaf].bound_box.zmin = std::min({ t.v1.z, t.v2.z, t.v3.z });
	leaf_nodes[leaf].bound_box.xmax = std::max({ t.v1.x, t.v2.x, t.v3.x });
	leaf_nodes[leaf].bound_box.ymax = std::max({ t.v1.y, t.v2.y, t.v3.y });
	leaf_nodes[leaf].bound_box.zmax = std::max({ t.v1.z, t.v2.z, t.v3.z });

	size_t indices_index = leaf_nodes_index_to_indices_index[leaf];
	size_t node_index = array_index_to_node_index(indices_index);
	size_t run = node_index;
	size_t down = node_index;
	AABB _a_b = getAABB(down);
	while (run != 0)
	{
		run = (run - 1) / 2;

		AABB& a_b = getAABB(run);
		if (2 * run + 1 == down)
		{
			AABB& _right = getAABB(2 * run + 2);
			a_b.xmin = std::min(_right.xmin, _a_b.xmin);
			a_b.ymin = std::min(_right.ymin, _a_b.ymin);
			a_b.zmin = std::min(_right.zmin, _a_b.zmin);
			a_b.xmax = std::max(_right.xmax, _a_b.xmax);
			a_b.ymax = std::max(_right.ymax, _a_b.ymax);
			a_b.zmax = std::max(_right.zmax, _a_b.zmax);
		}
		else
		{
			AABB& _left = getAABB(2 * run + 1);
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
pair<std::vector<pair<size_t, InterType>>, Value> BVHTree::searchInter(size_t cur, PointOrVector3D& p0, PointOrVector3D& d, Value tmin, Value tmax, const AABB& parent)  //要求tmin<=tmax
{
	pair<Value, Value> res;
	AABB _cur = getAABB(cur);

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

	if (cur < leaf_nodes.size() - 1)
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
		Triangle temp = leaf_nodes[indices[node_index_to_array_index(cur)]].t;
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

pair<std::vector<pair<size_t, InterType>>, Value> BVHTree::solveLineAndTriangleInter(PointOrVector3D& p0, PointOrVector3D& d, Value tmin, Value tmax)
{
	if (leaf_nodes.empty())
	{
		return { std::vector<pair<size_t, InterType>>(), Value(v_type::positive_inf) };
	}

	pair<std::vector<pair<size_t, InterType>>, Value> res = searchInter(0, p0, d, tmin, tmax, AABB());
	return res;
}

void BVHTree::CreateBVHTree(size_t cur) //cur从0开始
{
	if (cur < leaf_nodes.size() - 1)
	{
		size_t left = 2 * cur + 1;
		size_t right = 2 * cur + 2;
		CreateBVHTree(left);
		CreateBVHTree(right);

		AABB left_b_b = getAABB(left);
		AABB right_b_b = getAABB(right);

		AABB b_b;
		b_b.xmin = std::min(left_b_b.xmin, right_b_b.xmin);
		b_b.ymin = std::min(left_b_b.ymin, right_b_b.ymin);
		b_b.zmin = std::min(left_b_b.zmin, right_b_b.zmin);
		b_b.xmax = std::max(left_b_b.xmax, right_b_b.xmax);
		b_b.ymax = std::max(left_b_b.ymax, right_b_b.ymax);
		b_b.zmax = std::max(left_b_b.zmax, right_b_b.zmax);

		interval_nodes[cur].bound_box = b_b;
	}
}

BVHTree::BVHTree(const std::vector<Triangle>& _t)
{
	if (_t.empty())
	{
		std::cout << "三角形数组不能为空!" << std::endl;
		exit(-1);
	}

	using std::vector;
	new (&leaf_nodes) vector<BVHTreeNodeLeaf>(_t.size());
	new (&indices) std::vector<size_t>(_t.size());
	new (&interval_nodes) std::vector<BVHTreeNode>(_t.size() - 1);
	new (&leaf_nodes_index_to_indices_index) std::vector<size_t>(_t.size());

	low_ext = 1;
	while (low_ext << 1 <= _t.size() - 1)
	{
		low_ext <<= 1;
	}

	low_ext = _t.size() - 1 - (low_ext - 1);
	left_down_num = 2 * low_ext;
	right_down_num = _t.size() - left_down_num;

	for (size_t i = 0; i < _t.size(); ++i)
	{
		leaf_nodes[i].bound_box.xmin = std::min({ _t[i].v1.x, _t[i].v2.x, _t[i].v3.x });
		leaf_nodes[i].bound_box.ymin = std::min({ _t[i].v1.y, _t[i].v2.y, _t[i].v3.y });
		leaf_nodes[i].bound_box.zmin = std::min({ _t[i].v1.z, _t[i].v2.z, _t[i].v3.z });
		leaf_nodes[i].bound_box.xmax = std::max({ _t[i].v1.x, _t[i].v2.x, _t[i].v3.x });
		leaf_nodes[i].bound_box.ymax = std::max({ _t[i].v1.y, _t[i].v2.y, _t[i].v3.y });
		leaf_nodes[i].bound_box.zmax = std::max({ _t[i].v1.z, _t[i].v2.z, _t[i].v3.z });
		leaf_nodes[i].t = _t[i];
	}

	vector<RationalNumber> mid_array_x(_t.size());
	vector<RationalNumber> mid_array_y(_t.size());
	vector<RationalNumber> mid_array_z(_t.size());
	for (size_t i = 0; i < leaf_nodes.size(); ++i)
	{
		mid_array_x[i] = (leaf_nodes[i].bound_box.xmin + leaf_nodes[i].bound_box.xmax) / RationalNumber(2, 1);
		mid_array_y[i] = (leaf_nodes[i].bound_box.ymin + leaf_nodes[i].bound_box.ymax) / RationalNumber(2, 1);
		mid_array_z[i] = (leaf_nodes[i].bound_box.zmin + leaf_nodes[i].bound_box.zmax) / RationalNumber(2, 1);
	}

	for (size_t i = 0; i < _t.size(); ++i)
	{
		indices[i] = i;
	}

	adjustIndices(mid_array_x, mid_array_y, mid_array_z, 0, indices.size());
	CreateBVHTree(0);

	for (size_t i = 0; i < indices.size(); ++i)
	{
		leaf_nodes_index_to_indices_index[indices[i]] = i;
	}
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

	BVHTree test_tree(test_tri);

	PointOrVector3D _1p;
	_1p.x = RationalNumber(1, 2);
	_1p.y = RationalNumber(1, 2);
	_1p.z = RationalNumber(-1, 1);
	PointOrVector3D _1d;
	_1d.x = RationalNumber(0, 1);
	_1d.y = RationalNumber(0, 1);
	_1d.z = RationalNumber(1, 1);

	pair<std::vector<pair<size_t, InterType>>, Value> r1 = test_tree.solveLineAndTriangleInter(_1p, _1d, Value(r_zero), Value(v_type::positive_inf));
	assert(r1.second != v_type::positive_inf && r1.first.size() == 1 && test_tree.getLeaf(r1.first[0].first).t == t1 && r1.second.type == v_type::finite && r1.second.value == RationalNumber(1, 1) && "_1faild!");

	PointOrVector3D _2p;
	_2p.x = RationalNumber(1, 2);
	_2p.y = RationalNumber(1, 2);
	_2p.z = RationalNumber(1, 2);
	PointOrVector3D _2d;
	_2d.x = RationalNumber(0, 1);
	_2d.y = RationalNumber(0, 1);
	_2d.z = RationalNumber(1, 1);

	pair<std::vector<pair<size_t, InterType>>, Value> r2 = test_tree.solveLineAndTriangleInter(_2p, _2d, Value(r_zero), Value(v_type::positive_inf));
	assert(r2.second != v_type::positive_inf && r2.first.size() == 1 && test_tree.getLeaf(r2.first[0].first).t == t2 && r2.second.type == v_type::finite && r2.second.value == RationalNumber(1, 2) && "_2faild!");

	PointOrVector3D _3p;
	_3p.x = RationalNumber(5, 2);
	_3p.y = RationalNumber(1, 2);
	_3p.z = RationalNumber(-1, 1);
	PointOrVector3D _3d;
	_3d.x = RationalNumber(0, 1);
	_3d.y = RationalNumber(0, 1);
	_3d.z = RationalNumber(1, 1);

	pair<std::vector<pair<size_t, InterType>>, Value> r3 = test_tree.solveLineAndTriangleInter(_3p, _3d, Value(r_zero), Value(v_type::positive_inf));
	assert(r3.second != v_type::positive_inf && r3.first.size() == 1 && test_tree.getLeaf(r3.first[0].first).t == t3 && r3.second.type == v_type::finite && r3.second.value == RationalNumber(1, 1) && "_3faild!");

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
	assert(r5.second != v_type::positive_inf && r5.first.size() == 1 && test_tree.getLeaf(r5.first[0].first).t == t2 && r5.second.type == v_type::finite && r5.second.value == RationalNumber(1, 1) && "_5faild!");

	PointOrVector3D _6p;
	_6p.x = RationalNumber(10, 1);
	_6p.y = RationalNumber(10, 1);
	_6p.z = RationalNumber(10, 1);
	PointOrVector3D _6d;
	_6d.x = RationalNumber(-1, 1);
	_6d.y = RationalNumber(-1, 1);
	_6d.z = RationalNumber(-1, 1);

	pair<std::vector<pair<size_t, InterType>>, Value> r6 = test_tree.solveLineAndTriangleInter(_6p, _6d, Value(r_zero), Value(v_type::positive_inf));
	assert(r6.second != v_type::positive_inf && r6.first.size() == 1 && test_tree.getLeaf(r6.first[0].first).t == t1 && r6.second.type == v_type::finite && r6.second.value == RationalNumber(10, 1) && "_6faild!");

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

