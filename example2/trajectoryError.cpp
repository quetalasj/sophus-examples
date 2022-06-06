#include <iostream>
#include <fstream>
#include <unistd.h>
#include <pangolin/pangolin.h>
#include "sophus/se3.hpp"

using namespace std;
using namespace Sophus;

string GROUNDTRUTH_FILE = "../groundtruth.txt";
string ESTIMATED_FILE = "../estimated.txt";

typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;

void DrawTrajectory(const TrajectoryType& gt, const TrajectoryType& esti);

TrajectoryType ReadTrajectory(const string& path);

int main() {
    TrajectoryType groundtruth = ReadTrajectory(GROUNDTRUTH_FILE);
    TrajectoryType estimated = ReadTrajectory(ESTIMATED_FILE);

    assert(!groundtruth.empty() && !estimated.empty());
    assert(groundtruth.size() == estimated.size());

    double rmse = 0;
    for (size_t i = 0; i < estimated.size(); i++) {
        Sophus::SE3d p1 = estimated[i];
        Sophus::SE3d p2 = groundtruth[i];
        double error = (p2.inverse() * p1).log().norm();
        rmse += error * error;
    }
    rmse = rmse / double(estimated.size());
    rmse = sqrt(rmse);
    cout << "RMSE = " << rmse << endl;
    DrawTrajectory(groundtruth, estimated);
    return 0;
}

TrajectoryType ReadTrajectory(const string& path) {
    ifstream fin(path);
    TrajectoryType trajectory;
    if (!fin) {
        cerr << "trajectory " << path << " not found." << endl;
        return trajectory;
    }

    while (!fin.eof()) {
        double time, tx, ty, tz, qx, qy, qz, qw;
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Sophus::SE3d p1(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
        trajectory.push_back(p1);
    }
    return trajectory;
}

void DrawLine(const TrajectoryType& traj, const size_t& i) {
    glBegin(GL_LINES);
    auto p1 = traj[i], p2 = traj[i+1];
    glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
    glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
    glEnd();
}

void DrawTrajectory(const TrajectoryType& gt, const TrajectoryType& esti) {
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
            );
    pangolin::View& d_cam =pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/ 768.f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < gt.size() -1; i++) {
            glColor3d(0.0f, 0.0f, 1.0f);
            DrawLine(gt, i);
        }

        for (size_t i = 0; i < esti.size() -1; i++) {
            glColor3d(1.0f, 0.0f, 0.0f);
            DrawLine(esti, i);
        }

        pangolin::FinishFrame();
        usleep(5000);
    }
}