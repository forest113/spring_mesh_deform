#pragma once
#include <vector>
using namespace std;

#define PM_MASS 0.1       //mass of each vertex
#define SPRING_K 0.03    //spring constant
#define SPRING_DAMP 0.0000001 //damping force constant
#define NUM_ITR 150       //number of iterations to simulate


/* A 3D vector class. */
class Vector3 {
public:
	double x;
	double y;
	double z;

	Vector3() {}

	Vector3(double x, double y, double z) {
		this->x = x;
		this->y = y;
		this->z = z;
	}

	/* magnitude of the vector. */
	double mag() {
		return sqrt(pow(this->x, 2) + pow(this->y, 2) + pow(this->z, 2));
	}

	double dot(Vector3 v) {
		return (this->x * v.x + this->y * v.y + this->z * v.z);
	}

	void normalize() {
		double mag = this->mag();
		this->x /= mag;
		this->y /= mag;
		this->z /= mag;
	}

	void set_zero() {
		this->x = 0;
		this->y = 0;
		this->z = 0;
	}

	Vector3 operator * (double scalar) {
		return Vector3(this->x * scalar, this->y * scalar, this->z * scalar);
	}

	Vector3 operator / (double scalar) {
		return Vector3(this->x / scalar, this->y / scalar, this->z / scalar);
	}

	Vector3 operator - () {
		return Vector3(-this->x, -this->y, -this->z);
	}

	Vector3 operator + (Vector3 v) {
		return Vector3(this->x + v.x, this->y + v.y, this->z + v.z);
	}

	Vector3 operator - (Vector3 v) {
		return Vector3(this->x - v.x, this->y - v.y, this->z - v.z);
	}
};

/* A class to store vertices of the mesh. */
struct Point {
	Vector3 pos;
	bool is_fixed;
	Vector3 impulse;
	Vector3 vel;
	int num_springs;

	Point() {}

	Point(double px, double py, double pz) {
		this->pos = Vector3(px, py, pz);
		this->is_fixed = false;
		this->impulse = Vector3(0, 0, 0);
		this->vel = Vector3(0, 0, 0);
		this->num_springs = 0;
	}

};

/* function to get point - point distance. */
double distance(Point p1, Point p2) {
	double d = sqrt(pow(p2.pos.x - p1.pos.x, 2) +
		pow(p2.pos.y - p1.pos.y, 2) +
		pow(p2.pos.z - p1.pos.z, 2));
	return d;
}

/* Class to represent each of the springs between the vertices of the mesh. */
struct Spring {
	Point *p1;
	Point *p2;
	double rest_len;
	double cur_len;

	Spring() {}

	Spring(Point *p1, Point *p2) {
		this->p1 = p1;
		this->p2 = p2;
		if(p1 && p2){
		   this->rest_len = distance(*p1, *p2);
		   this->cur_len = this->rest_len;
	    }
	}

	Spring(const Spring& s) {
		this->p1 = s.p1;
		this->p2 = s.p2;
		this->rest_len = s.rest_len;
		this->cur_len - s.cur_len;
	}
};

/* reset impulse for each point to zero. */
void set_impulse_zero(vector<Point>& points_list) {
	for (int i = 0; i < points_list.size(); i++) {
		Point *p = &points_list[i];
		p->impulse.set_zero();
	}
}

/* check for length changes in the springs and apply the resulting impulses on the point masses. */
void process_springs(vector<Spring> &springs_list) {
	for (int i = 0; i < springs_list.size(); i++) {
		Spring* spring = &(springs_list[i]);
		Point* p1 = spring->p1;
		Point* p2 = spring->p2;
		spring->cur_len = distance(*p1, *p2);
		Vector3 spring_dir = Vector3(p1->pos.x - p2->pos.x, p1->pos.y - p2->pos.y, p1->pos.z - p2->pos.z);
		spring_dir.normalize();
		//cout << "p1 :" << p1->pos.x << " " << p1->pos.y << " " << p1->pos.z << endl;
		//cout << "p2 :" << p2->pos.x << " " << p2->pos.y << " " << p2->pos.z << endl;
		//cout << "spring dir:" << spring_dir.x << " " << spring_dir.y << " " << spring_dir.z<< endl;
		double x = spring->rest_len - spring->cur_len;
		double spring_impulse_mag = SPRING_K * x;
		float damping_mag = SPRING_DAMP * (p1->vel.dot(spring_dir) - p2->vel.dot(spring_dir));
		spring_impulse_mag += damping_mag;
		if (spring->rest_len < spring->cur_len) {
			//cout << "expanded" << endl;
			p1->impulse = p1->impulse + (spring_dir * spring_impulse_mag);
			p2->impulse = p2->impulse + (-spring_dir * spring_impulse_mag);
			
		}
		else if (spring->rest_len > spring->cur_len) {
			//cout << "contracted" << endl;
			p1->impulse = p1->impulse + (spring_dir * spring_impulse_mag);
			p2->impulse = p2->impulse + (-spring_dir * spring_impulse_mag);
			
		}
		//cout << "impulse:" << (-spring_dir * spring_impulse_mag).x << " " << (-spring_dir * spring_impulse_mag).y << " " << (-spring_dir * spring_impulse_mag).z << endl;
	}

}

/* update position and velocities of point masses according to the impulse acting on them. */
void update_points(float DeltaTime, vector<Point> &points_list) {
	for (int i = 0; i < points_list.size(); i++) {

		Point* p = &(points_list[i]);
		if (p->is_fixed) {
			continue;
		}
		/*update velocities*/
		if (p->impulse.mag() != 0.0) {
			p->vel = p->vel + (p->impulse / PM_MASS);
		}

		if (i == 5) {
			//cout << "impulse:" << p->impulse.x << " " << p->impulse.y << " " << p->impulse.z << endl;
			//cout << "vel" << p->vel.x << " " << p->vel.y << " " << p->vel.z << endl;
		}
		/* Euler integration for location*/
		//cout << "old:" << p->pos.x << " " << p->pos.y << " " << p->pos.z << endl;
		p->pos = p->pos + (p->vel * (DeltaTime));
		//cout << "new:" << p->pos.x << " " << p->pos.y << " " << p->pos.z << endl;
		
	}
}


