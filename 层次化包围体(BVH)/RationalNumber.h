#pragma once
#include <iostream>
#include <cmath>
#include <algorithm>

class RationalNumber
{
public:
	RationalNumber(long long m, long long d);
	RationalNumber() = default;
	long long computeGCD();
	bool operator<(const RationalNumber& opNum) const;
	bool operator<=(const RationalNumber& opNum) const;
	bool operator>(const RationalNumber& opNum) const;
	bool operator>=(const RationalNumber& opNum) const;
	bool operator==(const RationalNumber& opNum) const;
	bool operator!=(const RationalNumber& opNum) const;
	RationalNumber operator+(const RationalNumber& opNum) const;
	RationalNumber operator-(const RationalNumber& opNum) const;
	RationalNumber operator*(const RationalNumber& opNum) const;
	RationalNumber operator/(const RationalNumber& opNum) const;
	RationalNumber operator-() const { RationalNumber t = *this; t.positive = !t.positive; return t; }
	RationalNumber abs() const { RationalNumber t = *this; t.positive = true; return t; }
public:
	bool positive;
	long long molecule;
	long long denominator;
};

std::ostream& operator<<(std::ostream& out, const RationalNumber& output)
{
	if (output.positive == false && output.molecule != 0)
		out << "-";
	if (output.molecule == 0)
		out << "0";
	else
	{
		if (output.denominator == 1)
			out << output.molecule;
		else
			out << output.molecule << "/" << output.denominator;
	}
	return out;
}
bool RationalNumber::operator<(const RationalNumber& opNum) const
{
	if (molecule == 0)
	{
		if (opNum.molecule == 0)
		{
			return false;
		}
		else
		{
			if (opNum.positive)
				return true;
			return false;
		}
	}
	else
	{
		if (opNum.molecule == 0)
		{
			if (positive == false)
				return true;
			return false;
		}
		else
		{
			if (!positive && opNum.positive)
			{
				return true;
			}
			else if (positive && !opNum.positive)
			{
				return false;
			}
			else if (positive)
			{
				if (denominator == opNum.denominator)
				{
					return molecule < opNum.molecule;
				}
				else
				{
					if (molecule == opNum.molecule)
						return denominator > opNum.denominator;
					if (molecule < opNum.molecule && denominator > opNum.denominator)
						return true;
					if (molecule > opNum.molecule && denominator < opNum.denominator)
						return false;
					return molecule * opNum.denominator < opNum.molecule* denominator;
				}
			}
			else
			{
				if (denominator == opNum.denominator)
				{
					return molecule > opNum.molecule;
				}
				else
				{
					if (molecule == opNum.molecule)
						return denominator < opNum.denominator;
					if (molecule < opNum.molecule && denominator > opNum.denominator)
						return false;
					if (molecule > opNum.molecule && denominator < opNum.denominator)
						return true;
					return molecule * opNum.denominator > opNum.molecule * denominator;
				}
			}
		}
	}
}

bool RationalNumber::operator<=(const RationalNumber& opNum) const
{
	return !operator>(opNum);
}

bool RationalNumber::operator>(const RationalNumber& opNum) const
{
	if (molecule == 0)
	{
		if (opNum.molecule == 0)
		{
			return false;
		}
		else
		{
			if (opNum.positive)
				return false;
			return true;
		}
	}
	else
	{
		if (opNum.molecule == 0)
		{
			if (positive == false)
				return false;
			return true;
		}
		else
		{
			if (!positive && opNum.positive)
			{
				return false;
			}
			else if (positive && !opNum.positive)
			{
				return true;
			}
			else if (positive)
			{
				if (denominator == opNum.denominator)
				{
					return molecule > opNum.molecule;
				}
				else
				{
					if (molecule == opNum.molecule)
						return denominator < opNum.denominator;
					if (molecule < opNum.molecule && denominator > opNum.denominator)
						return false;
					if (molecule > opNum.molecule && denominator < opNum.denominator)
						return true;
					return molecule * opNum.denominator > opNum.molecule * denominator;
				}
			}
			else
			{
				if (denominator == opNum.denominator)
				{
					return molecule < opNum.molecule;
				}
				else
				{
					if (molecule == opNum.molecule)
						return denominator > opNum.denominator;
					if (molecule < opNum.molecule && denominator > opNum.denominator)
						return true;
					if (molecule > opNum.molecule && denominator < opNum.denominator)
						return false;
					return molecule * opNum.denominator < opNum.molecule* denominator;
				}
			}
		}
	}
}

bool RationalNumber::operator>=(const RationalNumber& opNum) const
{
	return !operator<(opNum);
}

bool RationalNumber::operator==(const RationalNumber& opNum) const
{
	if (molecule == 0)
	{
		if (opNum.molecule == 0)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		if (opNum.molecule == 0)
		{
			return false;
		}
		else
		{
			if (!positive && opNum.positive || positive && !opNum.positive)
			{
				return false;
			}
			else
			{
			    return denominator == opNum.denominator && opNum.molecule == molecule;
			}
		}
	}
}

bool RationalNumber::operator!=(const RationalNumber& opNum) const
{
	return !operator==(opNum);
}
RationalNumber RationalNumber::operator/(const RationalNumber& opNum) const
{
	if (opNum.molecule == 0)
	{
		std::cout << "Error,除数不能为零!" << std::endl;
		exit(-1);
	}

	if (molecule == 0)
		return RationalNumber(0, 1);

	RationalNumber temp = opNum;
	std::swap(temp.molecule, temp.denominator);
	return operator*(temp);
}

RationalNumber RationalNumber::operator*(const RationalNumber& opNum) const
{
	if (molecule == 0 || opNum.molecule == 0)
	{
		return RationalNumber(0, 1);
	}
	if (positive && !opNum.positive || !positive && opNum.positive)
	{
		return RationalNumber(-(molecule * opNum.molecule), denominator * opNum.denominator);
	}
	else
	{
		return RationalNumber(molecule * opNum.molecule, denominator * opNum.denominator);
	}
}

RationalNumber RationalNumber::operator-(const RationalNumber& opNum) const
{
	RationalNumber temp = opNum;
	temp.positive = !temp.positive;
	return operator+(temp);
}

RationalNumber RationalNumber::operator+(const RationalNumber& opNum) const
{
	if (molecule == 0)
	{
		return opNum;
	}

	if (opNum.molecule == 0)
	{
		return *this;
	}

	if (denominator == opNum.denominator)
	{
		if (positive)
		{
			if (opNum.positive)
			{
				return RationalNumber(molecule + opNum.molecule, denominator);
			}
			else
			{
				return RationalNumber(molecule - opNum.molecule, denominator);
			}
		}
		else
		{
			if (opNum.positive)
			{
				return RationalNumber(opNum.molecule - molecule, denominator);
			}
			else
			{
				return RationalNumber(-(opNum.molecule + molecule), denominator);
			}
		}
	}

	if (positive)
	{
		if (opNum.positive)
		{
			return RationalNumber(molecule * opNum.denominator + opNum.molecule * denominator, denominator * opNum.denominator);
		}
		else
		{
			return RationalNumber(molecule * opNum.denominator - opNum.molecule * denominator, denominator * opNum.denominator);
		}
	}
	else
	{
		if (opNum.positive)
		{
			return RationalNumber(opNum.molecule * denominator - molecule * opNum.denominator, denominator * opNum.denominator);
		}
		else
		{
			return RationalNumber(-(opNum.molecule * denominator + molecule * opNum.denominator), denominator * opNum.denominator);
		}
	}
}
long long RationalNumber::computeGCD()
{
	long long d = denominator;
	long long m = molecule;
	while (m != 0)
	{
		long long temp = m;
		m = d % m;
		d = temp;
	}
	return d;
}

RationalNumber::RationalNumber(long long m, long long d)
{
	if (d == 0)
	{
		std::cout << "ERROR,分母不能为零!" << std::endl;
		exit(-1);
	}
	if (m < 0 && d>0 || m > 0 && d < 0)
	{
		positive = false;
	}
	else
	{
		positive = true;
	}
	denominator = std::abs(d);
	molecule = std::abs(m);
	if (molecule != 0)
	{
		if (molecule > 1 && denominator > 1)
		{
			unsigned long long gcd = computeGCD();
			denominator /= gcd;
			molecule /= gcd;
		}
	}
}