#include <sophus/se3.h>
#include <math.h>
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pangolin/pangolin.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

// path to compare file
string compare_file = "../compare.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);
void GeneratePose(string&, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>&, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>&, vector<Point3f>&, vector<Point3f>&);
void ICP_SVD(const vector<Point3f>&, const vector<Point3f>&, Eigen::Matrix3d&, Eigen::Vector3d&);

int main(int argc, char **argv) {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_e;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_g;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_tg;
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    vector<Point3f> t_e,t_g;

    GeneratePose(compare_file, poses_e, poses_g, t_e, t_g);
    ICP_SVD(t_e, t_g, R, t);
    Sophus::SE3 T_eg(R,t);
    for(auto SE3_g:poses_g){
        SE3_g = T_eg * SE3_g;
        poses_tg.push_back(SE3_g);
    }
    DrawTrajectory(poses_e, poses_tg);
    
    return 0;
}

// Generate poses from the given files
void GeneratePose(string &file_name, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> &poses_e, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> &poses_g, vector<Point3f> &t_e, vector<Point3f> &t_g)
{
    string line;
    double time1, tx_1, ty_1, tz_1, qx_1, qy_1, qz_1, qw_1;
    double time2, tx_2, ty_2, tz_2, qx_2, qy_2, qz_2, qw_2;
    ifstream fin(file_name);
    if(!fin.is_open())
    {
        cout<<"compare.txt file can not open!"<<endl;
        return ;
    }
    
    while(getline(fin,line))
    {
        istringstream record(line);
        record>>time1 >> tx_1 >> ty_1 >> tz_1 >> qx_1 >> qy_1 >> qz_1 >> qw_1
              >>time2 >> tx_2 >> ty_2 >> tz_2 >> qx_2 >> qy_2 >> qz_2 >> qw_2;
        t_e.push_back(Point3d(tx_1,ty_1,tz_1));
        t_g.push_back(Point3d(tx_2,ty_2,tz_2));

        Eigen::Vector3d point_t1(tx_1, ty_1, tz_1);
        Eigen::Vector3d point_t2(tx_2, ty_2, tz_2);

        Eigen::Quaterniond q1 = Eigen::Quaterniond(qw_1, qx_1, qy_1, qz_1).normalized();
        Eigen::Quaterniond q2 = Eigen::Quaterniond(qw_2, qx_2, qy_2, qz_2).normalized();

        Sophus::SE3 SE3_qt1(q1, point_t1);
        Sophus::SE3 SE3_qt2(q2, point_t2);

        poses_e.push_back(SE3_qt1);
        poses_g.push_back(SE3_qt2);
    }
}

//
void ICP_SVD(const vector<Point3f>& pts1, const vector<Point3f>& pts2, Eigen::Matrix3d& R, Eigen::Vector3d& t) 
{
    Point3f p1, p2; // center of mass
    int N = pts1.size();
    for (int i = 0; i < N; i++)
    {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = Point3f(Vec3f(p1) / N);
    p2 = Point3f(Vec3f(p2) / N);
    vector<Point3f> q1(N), q2(N); // remove the center
    for (int i=0; i < N; i++)
    {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; i++)
    {
        W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
    }
    cout << "W=" << W << endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU|Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    cout << "U=" << U << endl;
    cout << "V=" << V <<endl;

    R = U * (V.transpose());
    t = Eigen::Vector3d(p1.x, p1.y, p1.z) - R * Eigen::Vector3d(p2.x, p2.y, p2.z);
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses1, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses2) 
{
    if (poses1.empty() || poses2.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Overlay", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses1.size() - 1; i++) {
            glColor3f(1 - (float) i / poses1.size(), 0.0f, (float) i / poses1.size());
            glBegin(GL_LINES);
            auto p11 = poses1[i], p12 = poses1[i + 1];
            glVertex3d(p11.translation()[0], p11.translation()[1], p11.translation()[2]);
            glVertex3d(p12.translation()[0], p12.translation()[1], p12.translation()[2]);
            glEnd();

            glColor3f(1 - (float) i / poses2.size(), 1.0f, (float) i / poses2.size());
            glBegin(GL_LINES);
            auto p21 = poses2[i], p22 = poses2[i + 1];
            glVertex3d(p21.translation()[0], p21.translation()[1], p21.translation()[2]);
            glVertex3d(p22.translation()[0], p22.translation()[1], p22.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}



