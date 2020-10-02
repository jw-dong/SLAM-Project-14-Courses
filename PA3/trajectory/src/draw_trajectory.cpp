#include <sophus/se3.h>
#include <math.h>
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Dense>
// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;

// path to trajectory file
string trajectory_file = "../src/trajectory.txt";
string groundtruth_file = "../src/groundtruth.txt";
string estimate_file = "../src/estimated.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);
void DrawTrajectory2(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);
vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> GeneratePose(string &file_name);

int main(int argc, char **argv) {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_ground;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_est;

    /// implement pose reading code
    // start your code here (5~10 lines)

	poses = GeneratePose(trajectory_file);
    // draw trajectory in pangolin
    DrawTrajectory(poses);
    
    // generate pose from groundtruth file
    poses_ground = GeneratePose(groundtruth_file);    
    
    // generate pose from estimate file
	poses_est = GeneratePose(estimate_file);
	
	DrawTrajectory2(poses_ground, poses_est);
    
    auto index_ground = poses_ground.begin();
    auto index_est = poses_est.begin();
    double error = 0.0;
    
    for(int i = 0; i < poses_ground.size() - 1; i++){
        error += Sophus::SE3::log(poses_ground[i].inverse() * poses_est[i]).squaredNorm();
    }
    error  = sqrt(error / (poses_ground.size() - 1));

    cout << error << endl;
    
    return 0;
}

// Generate poses from the given files
vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> GeneratePose(string &file_name){

    ifstream f(file_name);
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses;
    
    while (!f.eof())
    {
        double data[8] = { 0 };
        for (auto& d : data)
            f >> d;
        Eigen::Quaterniond q(data[7], data[4], data[5], data[6]);
        Eigen::Vector3d t(data[1], data[2], data[3]);
        Sophus::SE3 SE3_traj(q, t);
        poses.push_back(SE3_traj);
    }

	f.close();
	
	return poses;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses) {
    if (poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
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
        for (size_t i = 0; i < poses.size() - 1; i++) {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}

void DrawTrajectory2(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses1, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses2) {
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



