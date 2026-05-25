#include <iostream>
#include <utility>
#include <algorithm>
#include <vector>
#include <cassert>
#include "RationalNumber.h"

using std::pair;

RationalNumber r_zero(0, 1);

enum class v_type {positive_inf, negative_inf, finite};

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
pair<Value, Value> getIntersection(pair<Value, Value> l, pair<Value, Value> j, bool &inter) //返回区间l,j的交区间,l,g左右端点均为正整数,返回(0,0)表示交集不存在
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

RationalNumber compute_t(RationalNumber c, RationalNumber C, RationalNumber value, bool &inf)
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
	Value _x0 { x0 };
	Value _y0 { y0 };
	Value _z0 { z0 };
	Value _A { A };
	Value _B { B };
	Value _C { C };
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
	pair<Value, Value> x_inter = getIntersection({ x_range_left, x_range_right }, {_xmin, _xmax}, x_is_inter);
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
	PointOrVector3D() = default;
	bool operator==(const PointOrVector3D& be) const
	{
		return x == be.x && y == be.y && z == be.z;
	}
};

RationalNumber innerProduct(PointOrVector3D &v1, PointOrVector3D &v2)
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

enum class InterType { V1, V3, V2, V1V3, V3V2, V2V1, Internal, NOInter}; //三角形从V1出发逆时针依次为v1,v3,v2
pair<RationalNumber, InterType> lineInterTriangle(PointOrVector3D& p0, PointOrVector3D& d, Value tmin, Value tmax,PointOrVector3D& v1, PointOrVector3D& v2, PointOrVector3D& v3)
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
	bool operator==(const Triangle& be) const
	{
		return v1 == be.v1 && v2 == be.v2 && v3 == be.v3;
	}
};

struct BVHTreeNode;

struct PSubNode
{
	BVHTreeNode* left = nullptr;
	BVHTreeNode* right = nullptr;
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
	bool isleaf;
	AABB bound_box;
	BVHTreeNode* parent = nullptr;

	union
	{
		Triangle t;
		PSubNode p;
	};

	BVHTreeNode(const Triangle& t, const AABB &b) : isleaf(true), t(t), bound_box(b){}
	BVHTreeNode(const PSubNode& p, const AABB &b) : isleaf(false), p(p), bound_box(b){}
	BVHTreeNode(const BVHTreeNode& c) : isleaf(c.isleaf), bound_box(c.bound_box)
	{
		if (isleaf)
		{
			new (&t) Triangle(c.t);
		}
		else
		{
			PSubNode _p;
			new (&p) PSubNode(_p);
		}
	}
	~BVHTreeNode()
	{
		if (isleaf)
		{
			t.~Triangle();
		}
		else
		{
			p.~PSubNode();
		}
	}
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
	BVHTree() : root(nullptr) {}
	BVHTree(const std::vector<Triangle>& _t);
	BVHTree(const BVHTree& b);
	BVHTree(BVHTree&& b);
	void exchange(BVHTree& s) { std::swap(root, s.root); }
	const BVHTree & operator=(const BVHTree& b)
	{
		if (this != &b)
		{
			this->~BVHTree();
			BVHTree temp(b);
			root = temp.root;
			temp.root = nullptr;
		}
		return *this;
	}

	const BVHTree& operator=(BVHTree&& b)
	{
		if (this != &b)
		{
			this->~BVHTree();
			root = b.root;
			b.root = nullptr;
		}
		return *this;
	}

	~BVHTree();
	pair<std::vector<pair<BVHTreeNode*, InterType>>, Value> solveLineAndTriangleInter(PointOrVector3D& p0, PointOrVector3D& d, Value tmin, Value tmax);
	void update(BVHTreeNode *leaf, const Triangle& t);
private:
	void deleteBVHTree(BVHTreeNode* cur);
	BVHTreeNode* copy(BVHTreeNode* cur);
	BVHTreeNode* CreateBVHTree(const std::vector<Triangle>& _t, std::vector<AABB>& box_array, std::vector<size_t>& indices, std::vector<RationalNumber>& mid_array_x,
		std::vector<RationalNumber>& mid_array_y, std::vector<RationalNumber>& mid_array_z, size_t left, size_t right);
	pair<std::vector<pair<BVHTreeNode*, InterType>>, Value> searchInter(BVHTreeNode *cur, PointOrVector3D& p0, PointOrVector3D& d, Value tmin, Value tmax, const AABB& current);
	BVHTreeNode* root;
};

void BVHTree::update(BVHTreeNode* leaf, const Triangle &t)
{
	if (leaf->t == t)
		return;
	leaf->t = t;
	leaf->bound_box.xmin = std::min({ t.v1.x, t.v2.x, t.v3.x });
	leaf->bound_box.ymin = std::min({ t.v1.y, t.v2.y, t.v3.y });
	leaf->bound_box.zmin = std::min({ t.v1.z, t.v2.z, t.v3.z });
	leaf->bound_box.xmax = std::max({ t.v1.x, t.v2.x, t.v3.x });
	leaf->bound_box.ymax = std::max({ t.v1.y, t.v2.y, t.v3.y });
	leaf->bound_box.zmax = std::max({ t.v1.z, t.v2.z, t.v3.z });

	BVHTreeNode* run = leaf;
	BVHTreeNode* down = leaf;
	while (run->parent != nullptr)
	{
		run = run->parent;

		if (run->p.left == down)
		{
			run->bound_box.xmin = std::min(run->p.right->bound_box.xmin, down->bound_box.xmin);
			run->bound_box.ymin = std::min(run->p.right->bound_box.ymin, down->bound_box.ymin);
			run->bound_box.zmin = std::min(run->p.right->bound_box.zmin, down->bound_box.zmin);
			run->bound_box.xmax = std::max(run->p.right->bound_box.xmax, down->bound_box.xmax);
			run->bound_box.ymax = std::max(run->p.right->bound_box.ymax, down->bound_box.ymax);
			run->bound_box.zmax = std::max(run->p.right->bound_box.zmax, down->bound_box.zmax);
		}
		else
		{
			run->bound_box.xmin = std::min(run->p.left->bound_box.xmin, down->bound_box.xmin);
			run->bound_box.ymin = std::min(run->p.left->bound_box.ymin, down->bound_box.ymin);
			run->bound_box.zmin = std::min(run->p.left->bound_box.zmin, down->bound_box.zmin);
			run->bound_box.xmax = std::max(run->p.left->bound_box.xmax, down->bound_box.xmax);
			run->bound_box.ymax = std::max(run->p.left->bound_box.ymax, down->bound_box.ymax);
			run->bound_box.zmax = std::max(run->p.left->bound_box.zmax, down->bound_box.zmax);
		}
		down = run;
	}
}

BVHTree::BVHTree(BVHTree&& b)
{
	root = b.root;
	b.root = nullptr;
}

BVHTreeNode* BVHTree::copy(BVHTreeNode* cur)
{
	BVHTreeNode* r = nullptr;
	try
	{
		r = new BVHTreeNode(*cur);
		if (cur->isleaf == false)
		{
			r->p.left = copy(cur->p.left);
			r->p.right = copy(cur->p.right);
		}
		return r;
	}
	catch (const std::bad_alloc& e)
	{
		delete r;
		throw e;
	}
}

BVHTree::BVHTree(const BVHTree& b)
{
	if (b.root == nullptr)
	{
		root = nullptr;
	}
	else
	{
		try
		{
			root = copy(b.root);
		}
		catch (const std::bad_alloc& e)
		{
			root = nullptr;
		}
	}
}

void BVHTree::deleteBVHTree(BVHTreeNode* cur)
{
	if (cur->isleaf)
	{
		delete cur;
		return;
	}

	deleteBVHTree(cur->p.left);
	deleteBVHTree(cur->p.right);
	delete cur;
}

BVHTree::~BVHTree()
{
	if (root != nullptr)
	{
		deleteBVHTree(root);
		root = nullptr;
	}
}

//Value是最近相交点的p+t*d的t值,vector<pair(存放和线段相交的三角形所在BVH树叶节点的指针,和三角形的相交类型)>
pair<std::vector<pair<BVHTreeNode*, InterType>>, Value> BVHTree::searchInter(BVHTreeNode* cur, PointOrVector3D& p0, PointOrVector3D& d, Value tmin, Value tmax, const AABB &parent)  //要求tmin<=tmax
{
	pair<Value, Value> res;
	if (!(cur != root && parent.xmin == cur->bound_box.xmin && parent.ymin == cur->bound_box.ymin && parent.zmin == cur->bound_box.zmin
		&& parent.xmax == cur->bound_box.xmax && parent.ymax == cur->bound_box.ymax && parent.zmax == cur->bound_box.zmax))
	{
		res = judgeAABBInterline(cur->bound_box.xmin, cur->bound_box.ymin, cur->bound_box.zmin, cur->bound_box.xmax, cur->bound_box.ymax, cur->bound_box.zmax,
			p0.x, p0.y, p0.z, d.x, d.y, d.z, tmin.value, tmax.value, tmin.type, tmax.type);

		if (res.first == v_type::positive_inf)
		{
			return { std::vector<pair<BVHTreeNode*, InterType>>(), Value(v_type::positive_inf)};
		}
	}
	else
	{
		res = { tmin, tmax };
	}

	if (cur->isleaf == false)
	{
		pair<std::vector<pair<BVHTreeNode*, InterType>>, Value> _left = searchInter(cur->p.left, p0, d, res.first, res.second, cur->bound_box);
		pair<std::vector<pair<BVHTreeNode*, InterType>>, Value> _right = searchInter(cur->p.right, p0, d, res.first, res.second, cur->bound_box);

		if (_left.second.type == v_type::positive_inf)
		{
			if (_right.second.type == v_type::finite)
			{
				return _right;
			}
			else
			{
				return { std::vector<pair<BVHTreeNode*, InterType>>(), Value(v_type::positive_inf)};
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
		pair<RationalNumber, InterType> _res = lineInterTriangle(p0, d, res.first, res.second, cur->t.v1, cur->t.v2, cur->t.v3);
		if (_res.second != InterType::NOInter)
		{
			std::vector<pair<BVHTreeNode*, InterType>> _r(1);
			_r[0] = { cur, _res.second };
			return { _r, Value(_res.first) };
		}
		else
		{
			return { std::vector<pair<BVHTreeNode*, InterType>>(), Value(v_type::positive_inf) };
		}
	}
}

pair<std::vector<pair<BVHTreeNode*, InterType>>, Value> BVHTree::solveLineAndTriangleInter(PointOrVector3D& p0, PointOrVector3D& d, Value tmin, Value tmax)
{
	if (root == nullptr)
	{
		return { std::vector<pair<BVHTreeNode*, InterType>>(), Value(v_type::positive_inf) };
	}

	pair<std::vector<pair<BVHTreeNode*, InterType>>, Value> res = searchInter(root, p0, d, tmin, tmax, AABB());
	return res;
}

BVHTreeNode* BVHTree::CreateBVHTree(const std::vector<Triangle>& _t, std::vector<AABB> &box_array, std::vector<size_t> &indices, std::vector<RationalNumber> &mid_array_x,
	std::vector<RationalNumber>& mid_array_y, std::vector<RationalNumber>& mid_array_z, size_t left, size_t right) //left是起始,right是终止+1
{
	if (right - left == 1)
	{
		AABB b_b = box_array[indices[left]];
		BVHTreeNode* leaf = new BVHTreeNode(_t[indices[left]], b_b);
		return leaf;
	}
	else
	{
		RationalNumber x_minest = box_array[indices[left]].xmin;
		RationalNumber x_maxest = box_array[indices[left]].xmax;
		RationalNumber y_minest = box_array[indices[left]].ymin;
		RationalNumber y_maxest = box_array[indices[left]].ymax;
		RationalNumber z_minest = box_array[indices[left]].zmin;
		RationalNumber z_maxest = box_array[indices[left]].zmax;
		for (size_t i = left + 1; i < right; ++i)
		{
			x_minest = std::min(x_minest, box_array[indices[i]].xmin);
			x_maxest = std::max(x_maxest, box_array[indices[i]].xmax);
			y_minest = std::min(y_minest, box_array[indices[i]].ymin);
			y_maxest = std::max(y_maxest, box_array[indices[i]].ymax);
			z_minest = std::min(z_minest, box_array[indices[i]].zmin);
			z_maxest = std::max(z_maxest, box_array[indices[i]].zmax);
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
		BVHTreeNode* left_p = CreateBVHTree(_t, box_array, indices, mid_array_x, mid_array_y, mid_array_z, left, mid + 1);
		BVHTreeNode* right_p = CreateBVHTree(_t, box_array, indices, mid_array_x, mid_array_y, mid_array_z, mid + 1, right);

		PSubNode p_sub;
		p_sub.left = left_p;
		p_sub.right = right_p;
		AABB b_b;
		b_b.xmin = std::min(left_p->bound_box.xmin, right_p->bound_box.xmin);
		b_b.ymin = std::min(left_p->bound_box.ymin, right_p->bound_box.ymin);
		b_b.zmin = std::min(left_p->bound_box.zmin, right_p->bound_box.zmin);
		b_b.xmax = std::max(left_p->bound_box.xmax, right_p->bound_box.xmax);
		b_b.ymax = std::max(left_p->bound_box.ymax, right_p->bound_box.ymax);
		b_b.zmax = std::max(left_p->bound_box.zmax, right_p->bound_box.zmax);
		BVHTreeNode* parent = new BVHTreeNode(p_sub, b_b);
		left_p->parent = parent;
		right_p->parent = parent;
		return parent;
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
	vector<AABB> box_array(_t.size());
	vector<size_t> indices(_t.size());
	vector<RationalNumber> mid_array_x(_t.size());
	vector<RationalNumber> mid_array_y(_t.size());
	vector<RationalNumber> mid_array_z(_t.size());
	for (size_t i = 0; i < _t.size(); ++i)
	{
		box_array[i].xmin = std::min({ _t[i].v1.x, _t[i].v2.x, _t[i].v3.x });
		box_array[i].ymin = std::min({ _t[i].v1.y, _t[i].v2.y, _t[i].v3.y });
		box_array[i].zmin = std::min({ _t[i].v1.z, _t[i].v2.z, _t[i].v3.z });
		box_array[i].xmax = std::max({ _t[i].v1.x, _t[i].v2.x, _t[i].v3.x });
		box_array[i].ymax = std::max({ _t[i].v1.y, _t[i].v2.y, _t[i].v3.y });
		box_array[i].zmax = std::max({ _t[i].v1.z, _t[i].v2.z, _t[i].v3.z });
	}

	for (size_t i = 0; i < box_array.size(); ++i)
	{
		mid_array_x[i] = (box_array[i].xmin + box_array[i].xmax) / RationalNumber(2, 1);
		mid_array_y[i] = (box_array[i].ymin + box_array[i].ymax) / RationalNumber(2, 1);
		mid_array_z[i] = (box_array[i].zmin + box_array[i].zmax) / RationalNumber(2, 1);
	}

	for (size_t i = 0; i < box_array.size(); ++i)
	{
		indices[i] = i;
	}

	root = CreateBVHTree(_t, box_array, indices, mid_array_x, mid_array_y, mid_array_z, 0, _t.size());
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

	pair<std::vector<pair<BVHTreeNode*, InterType>>, Value> r1 = test_tree.solveLineAndTriangleInter(_1p, _1d, Value(r_zero), Value(v_type::positive_inf));
	assert(r1.second != v_type::positive_inf && r1.first.size() == 1 && r1.first[0].first->t == t1 && r1.second.type == v_type::finite && r1.second.value == RationalNumber(1, 1) && "_1faild!");

	PointOrVector3D _2p;
	_2p.x = RationalNumber(1, 2);
	_2p.y = RationalNumber(1, 2);
	_2p.z = RationalNumber(1, 2);
	PointOrVector3D _2d;
	_2d.x = RationalNumber(0, 1);
	_2d.y = RationalNumber(0, 1);
	_2d.z = RationalNumber(1, 1);

	pair<std::vector<pair<BVHTreeNode*, InterType>>, Value> r2 = test_tree.solveLineAndTriangleInter(_2p, _2d, Value(r_zero), Value(v_type::positive_inf));
	assert(r2.second != v_type::positive_inf && r2.first.size() == 1 && r2.first[0].first->t == t2 && r2.second.type == v_type::finite && r2.second.value == RationalNumber(1, 2) && "_2faild!");

	PointOrVector3D _3p;
	_3p.x = RationalNumber(5, 2);
	_3p.y = RationalNumber(1, 2);
	_3p.z = RationalNumber(-1, 1);
	PointOrVector3D _3d;
	_3d.x = RationalNumber(0, 1);
	_3d.y = RationalNumber(0, 1);
	_3d.z = RationalNumber(1, 1);

	pair<std::vector<pair<BVHTreeNode*, InterType>>, Value> r3 = test_tree.solveLineAndTriangleInter(_3p, _3d, Value(r_zero), Value(v_type::positive_inf));
	assert(r3.second != v_type::positive_inf && r3.first.size() == 1 && r3.first[0].first->t == t3 && r3.second.type == v_type::finite && r3.second.value == RationalNumber(1, 1) && "_3faild!");

	PointOrVector3D _4p;
	_4p.x = RationalNumber(1, 2);
	_4p.y = RationalNumber(-1, 1);
	_4p.z = RationalNumber(0, 1);
	PointOrVector3D _4d;
	_4d.x = RationalNumber(0, 1);
	_4d.y = RationalNumber(1, 1);
	_4d.z = RationalNumber(0, 1);

	pair<std::vector<pair<BVHTreeNode*, InterType>>, Value> r4 = test_tree.solveLineAndTriangleInter(_4p, _4d, Value(r_zero), Value(v_type::positive_inf));
	assert(r4.second == v_type::positive_inf && "_4faild!");

	PointOrVector3D _5p;
	_5p.x = RationalNumber(1, 2);
	_5p.y = RationalNumber(1, 2);
	_5p.z = RationalNumber(2, 1);
	PointOrVector3D _5d;
	_5d.x = RationalNumber(0, 1);
	_5d.y = RationalNumber(0, 1);
	_5d.z = RationalNumber(-1, 1);

	pair<std::vector<pair<BVHTreeNode*, InterType>>, Value> r5 = test_tree.solveLineAndTriangleInter(_5p, _5d, Value(r_zero), Value(v_type::positive_inf));
	assert(r5.second != v_type::positive_inf && r5.first.size() == 1 && r5.first[0].first->t == t2 && r5.second.type == v_type::finite && r5.second.value == RationalNumber(1, 1) && "_5faild!");

	PointOrVector3D _6p;
	_6p.x = RationalNumber(10, 1);
	_6p.y = RationalNumber(10, 1);
	_6p.z = RationalNumber(10, 1);
	PointOrVector3D _6d;
	_6d.x = RationalNumber(-1, 1);
	_6d.y = RationalNumber(-1, 1);
	_6d.z = RationalNumber(-1, 1);

	pair<std::vector<pair<BVHTreeNode*, InterType>>, Value> r6 = test_tree.solveLineAndTriangleInter(_6p, _6d, Value(r_zero), Value(v_type::positive_inf));
	assert(r6.second != v_type::positive_inf && r6.first.size() == 1 && r6.first[0].first->t == t1 && r6.second.type == v_type::finite && r6.second.value == RationalNumber(10, 1) && "_6faild!");

	PointOrVector3D _7p;
	_7p.x = RationalNumber(1, 2);
	_7p.y = RationalNumber(1, 2);
	_7p.z = RationalNumber(1, 2);
	PointOrVector3D _7d;
	_7d.x = RationalNumber(1, 1);
	_7d.y = RationalNumber(0, 1);
	_7d.z = RationalNumber(0, 1);

	pair<std::vector<pair<BVHTreeNode*, InterType>>, Value> r7 = test_tree.solveLineAndTriangleInter(_7p, _7d, Value(r_zero), Value(v_type::positive_inf));
	assert(r7.second == v_type::positive_inf && "_7faild!");

	PointOrVector3D _8p;
	_8p.x = RationalNumber(4, 5);
	_8p.y = RationalNumber(4, 5);
	_8p.z = RationalNumber(-1, 1);
	PointOrVector3D _8d;
	_8d.x = RationalNumber(0, 1);
	_8d.y = RationalNumber(0, 1);
	_8d.z = RationalNumber(1, 1);

	pair<std::vector<pair<BVHTreeNode*, InterType>>, Value> r8 = test_tree.solveLineAndTriangleInter(_8p, _8d, Value(r_zero), Value(v_type::positive_inf));
	assert(r8.second == v_type::positive_inf && "_8faild!");

	PointOrVector3D _9p;
	_9p.x = RationalNumber(4, 5);
	_9p.y = RationalNumber(4, 5);
	_9p.z = RationalNumber(1, 2);
	PointOrVector3D _9d;
	_9d.x = RationalNumber(0, 1);
	_9d.y = RationalNumber(0, 1);
	_9d.z = RationalNumber(1, 1);

	pair<std::vector<pair<BVHTreeNode*, InterType>>, Value> r9 = test_tree.solveLineAndTriangleInter(_9p, _9d, Value(r_zero), Value(v_type::positive_inf));
	assert(r9.second == v_type::positive_inf && "_9faild!");
    return 0;

}

