#include <bits/stdc++.h>
#include <eigen3/Eigen/Dense>
#define err 0.000001

using namespace std;
using namespace Eigen;

struct Point{
	int x;
	int y;
};

class Quad{
private:
	MatrixXd X;
	MatrixXd J;
	Point *data_set;
	int no_points;

public:
	Quad(Point *data,int n):X(3,1),J(3,1)
	{
		data_set=data;
		X<<0,0,0;
		no_points=n;
	}

	float get_val(int p)
	{
		float val;
		val=X(0.0) + X(1.0)*p + X(2.0)*p*p;
		return val;
	}

	void compute_J()
	{
		float A=0,B=0,C=0;
		for(int i=0;i<no_points;i++)
		{
			A+=(get_val(data_set[i].x)-data_set[i].y);
			B+=(get_val(data_set[i].x)-data_set[i].y)*data_set[i].x;
			C+=(get_val(data_set[i].x)-data_set[i].y)*data_set[i].x*data_set[i].x;
		}
		A/=no_points;
		B/=no_points;
		C/=no_points;
		J<<A,B,C;
		cout << J<<'\n';
	}

	bool check_equality(MatrixXd A, MatrixXd B)
	{
		if(A(0.0)-B(0.0)>err || A(0.0)-B(0.0)<-err)return 0;
		if(A(1.0)-B(1.0)>err || A(1.0)-B(1.0)<-err)return 0;
		if(A(2.0)-B(2.0)>err || A(2.0)-B(2.0)<-err)return 0;
		return 1;

	}

	void grad_descent(MatrixXd X_old,float a)
	{
		
		compute_J();
		X=X_old-a*J;
		if (check_equality(X,X_old))
			return;
		else
			grad_descent(X,a);

	}
	void calc(float a)
	{
		grad_descent(X,a);
	}
	void display()
	{
		cout<<X<<'\n';
	}
};

int main()
{
	Point data[50];
	int size;
	cout << " Enter the size  ";
	cin>>size;
	cout << " ENter the points: \n";
	for (int i=0;i<size;i++)
		cin>>data[i].x>>data[i].y;
	Quad Q(data,size);
	Q.calc(0.01);
	Q.display();
	return 0;
}