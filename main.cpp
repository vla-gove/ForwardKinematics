#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

const double pi = 3.14159;

struct Joint {
    int type; // 0 for revolute, 1 for prismatic
    double theta;
    double d;
    double a;
    double alpha;
};

vector<Joint> joints;

void inputRobot() {
    int dof;
    cout << "degrees of freedom?";
    cin >> dof;

    for (int i = 0; i < dof; i++) {
        Joint j;
        cout << "joint " << i+1 << " type? (0 for revolute, 1 for prismatic)";
        cin >> j.type;
        if (j.type == 0) {
            cout << "input theta " << i+1 << ":";
            cin >> j.theta;
        } else {
            cout << "input d " << i+1 << ":";
            cin >> j.d;
        }
        cout << "input a " << i+1 << ":";
        cin >> j.a;
        cout << "input alpha " << i+1 << ":";
        cin >> j.alpha;
        joints.push_back(j);
    }
}

void forward_kinematics(vector<Joint> joints, double& x, double& y, double& z) {
    //tranformation matrix of the base frame to itself
    double Tp[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    //calculating homogenous transformation matrices for each joint
    double T[4][4];
    for (int i = 0; i < joints.size(); i++) {
        if (joints[i].type == 0) {
            //transformation matrix parameters of a rotational joint
            T[0][0] = cos(joints[i].theta);
            T[0][1] = -sin(joints[i].theta)*cos(joints[i].alpha);
            T[0][2] = sin(joints[i].theta)*sin(joints[i].alpha);
            T[0][3] = joints[i].a*cos(joints[i].theta);
            T[1][0] = sin(joints[i].theta);
            T[1][1] = cos(joints[i].theta)*cos(joints[i].alpha);
            T[1][2] = -cos(joints[i].theta)*sin(joints[i].alpha);
            T[1][3] = joints[i].a*sin(joints[i].theta);
        } else {
            //transformation matrix parameters of a prismatic joint
            T[0][0] = 1;
            T[0][1] = 0;
            T[0][2] = 0;
            T[0][3] = joints[i].d;
            T[1][0] = 0;
            T[1][1] = 1;
            T[1][2] = 0;
            T[1][3] = 0;
        }
        //transformation matrix parameters for both rotational and prismatic joints
        T[2][0] = 0;
        T[2][1] = sin(joints[i].alpha);
        T[2][2] = cos(joints[i].alpha);
        T[2][3] = joints[i].d;
        T[3][0] = 0;
        T[3][1] = 0;
        T[3][2] = 0;
        T[3][3] = 1;

        //multiplication between the Tp matrix and the current T matrix stored in Ttemp
        double Ttemp[4][4];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                Ttemp[i][j] = Tp[i][0]*T[0][j] + Tp[i][1]*T[1][j] + Tp[i][2]*T[2][j] + Tp[i][3]*T[3][j];
            }
        }
        //copying the values of Ttemp matrix into Tp matrix
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                Tp[i][j] = Ttemp[i][j];
            }
        }
    }
    x = Tp[0][3];
    y = Tp[1][3];
    z = Tp[2][3];
}

int main() {
    //defining the robot structure and parameters
    inputRobot();
    double x, y, z;
    //calculating forward kinematics and printing the end effector position
    forward_kinematics(joints, x, y, z);
    cout << "end effector position is: (" << x << ", " << y << ", " << z << ")" << endl;
    return 0;
}