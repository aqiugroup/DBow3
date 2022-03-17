#ifndef __PX_MAPPING_LOCALIZATION_EXTRINSIC_PARAMS_H__
#define __PX_MAPPING_LOCALIZATION_EXTRINSIC_PARAMS_H__
#include "file_utils.h"
#include "string_utils.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class IntrinsicParam {
  public:
    std::string         camera_id_;
    Eigen::Matrix3d     intrinsics_;
    std::vector<double> distortion_coeffs_;
    int                 width_  = 0;
    int                 height_ = 0;
    std::string         topic_  = "";

    float GetFx() const;
    float GetFy() const;
    float GetCx() const;
    float GetCy() const;

    std::string GetCameraId() const;
    int         GetWidth() const;
    int         GetHeight() const;

    Eigen::Matrix3d GetIntrinsic();

    bool ScaleTo(float s, IntrinsicParam& new_intrisic);

    void Print();
};

class ExtrinsicParam {
  public:
    ExtrinsicParam();
    ExtrinsicParam(std::string fn, std::string tn, Eigen::Matrix3d rot, Eigen::Vector3d trans);

    bool               IsValid();
    Eigen::Quaterniond GetQua();
    Eigen::Matrix3d    GetRot();
    Eigen::Vector3d    GetTrans();
    Eigen::Vector3d    GetRPY();
    Eigen::Matrix4d    GetT();
    Eigen::Matrix4d    GetTInverse();

    bool IsMatch(std::string fn, std::string tn);

    std::string GetFromName() const;
    std::string GetToName() const;
    void        Print();

  private:
    std::string from_name_ = "";
    std::string to_name_   = "";

    Eigen::Matrix4d    T_, T_inv_;
    Eigen::Matrix3d    rot_;
    Eigen::Vector3d    trans_;
    Eigen::Quaterniond qua_;
    Eigen::Vector3d    rpy_;
};

class ExtrinsicParamManager {
  private:
    std::mutex                            mtx_;
    std::map<std::string, ExtrinsicParam> extrinc_params_;    // code to params
    std::map<std::string, IntrinsicParam> intrinsic_params_;  // camera name to intrinsic params

  public:
    void Parse(std::string config_file);
    // return code if succ
    std::string AddExtrinsicParam(ExtrinsicParam ep);
    bool        GetExtrinsicParamByName(std::string fn, std::string tn, ExtrinsicParam& res);
    bool        GetExtrinsicParamByCode(std::string code, ExtrinsicParam& param);
    bool        FindCameraIntrinsicByCameraId(std::string camera_id, IntrinsicParam& camera_intrinsic);
    bool        DeleteByCode(std::string code);
    void        Clear();
    void        Print();

  private:
    std::string GenerateExtrinsicPairCode(std::string fn, std::string tn);
};

#endif