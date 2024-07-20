#pragma once
#include <Eigen/Dense>

class evo_tool
{
public:
    evo_tool(const std::string &trajectory_path)
    {
        pose_trajectory = fopen(trajectory_path.c_str(), "w");
        fprintf(pose_trajectory, "# target trajectory\n# timestamp tx ty tz qx qy qz qw\n");
        fflush(pose_trajectory);
    }
    ~evo_tool()
    {
        fclose(pose_trajectory);
    }

    void save_trajectory(const Eigen::Vector3d &pos, const Eigen::Quaterniond &quat, const double &time)
    {
        fprintf(pose_trajectory, "%0.4lf %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f\n", time,
                pos.x(), pos.y(), pos.z(), quat.x(), quat.y(), quat.z(), quat.w());
        fflush(pose_trajectory);
    }

    FILE *pose_trajectory;
};

template <typename T>
static Eigen::Matrix<T, 3, 1> RotationMatrix2RPY(const Eigen::Matrix<T, 3, 3> &rotation)
{
    // return rotation_matrix.eulerAngles(0, 1, 2);

    // fix eigen bug: https://blog.csdn.net/qq_36594547/article/details/119218807
    const Eigen::Matrix<T, 3, 1> &n = rotation.col(0);
    const Eigen::Matrix<T, 3, 1> &o = rotation.col(1);
    const Eigen::Matrix<T, 3, 1> &a = rotation.col(2);

    Eigen::Matrix<T, 3, 1> rpy(3);
    const double &y = atan2(n(1), n(0));
    const double &p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    const double &r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    rpy(0) = r;
    rpy(1) = p;
    rpy(2) = y;
    return rpy;
}

template <typename T>
static Eigen::Quaternion<T> RPY2Quaternion(const Eigen::Matrix<T, 3, 1> &eulerAngles)
{
    Eigen::AngleAxis<T> rollAngle(AngleAxis<T>(eulerAngles(0), Matrix<T, 3, 1>::UnitX()));
    Eigen::AngleAxis<T> pitchAngle(AngleAxis<T>(eulerAngles(1), Matrix<T, 3, 1>::UnitY()));
    Eigen::AngleAxis<T> yawAngle(AngleAxis<T>(eulerAngles(2), Matrix<T, 3, 1>::UnitZ()));
    Eigen::Quaternion<T> quaternion;
    quaternion = yawAngle * pitchAngle * rollAngle;
    return quaternion;
}

template <typename T>
static Eigen::Matrix<T, 3, 1> Quaternion2RPY(const Eigen::Quaternion<T> &quaternion)
{
    return RotationMatrix2RPY(quaternion.normalized().toRotationMatrix());
}

/**
 * @brief transform frame_a to frame_b
 * @param extR rot from frame_b to frame_a
 * @param extP pos from frame_b to frame_a
 */
template <typename T>
void poseTransformFrame(const Eigen::Quaternion<T> &rot_from, const Eigen::Matrix<T, 3, 1> &pos_from,
                        const Eigen::Quaternion<T> &extR, const Eigen::Matrix<T, 3, 1> &extP,
                        Eigen::Quaternion<T> &rot_to, Eigen::Matrix<T, 3, 1> &pos_to)
{
    rot_to = rot_from * extR;
    pos_to = rot_from * extP + pos_from;
}

/**
 * @brief transform frame_a to frame_b
 * @param extR rot from frame_a to frame_b
 * @param extP pos from frame_a to frame_b
 */
template <typename T>
void poseTransformFrame2(const Eigen::Quaternion<T> &rot_from, const Eigen::Matrix<T, 3, 1> &pos_from,
                         const Eigen::Quaternion<T> &extR, const Eigen::Matrix<T, 3, 1> &extP,
                         Eigen::Quaternion<T> &rot_to, Eigen::Matrix<T, 3, 1> &pos_to)
{
    rot_to = rot_from * extR.conjugate();
    pos_to = pos_from - rot_to * extP;
}
