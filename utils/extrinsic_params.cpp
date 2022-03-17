#include "extrinsic_params.h"

ExtrinsicParam::ExtrinsicParam()
{
    T_.setIdentity();
    rot_   = T_.block<3, 3>(0, 0);
    trans_ = T_.block<3, 1>(0, 3);

    qua_ = Eigen::Quaterniond(rot_);

    T_inv_ = T_.inverse();
}

ExtrinsicParam::ExtrinsicParam(std::string fn, std::string tn, Eigen::Matrix3d rot, Eigen::Vector3d trans)
{
    from_name_ = fn;
    to_name_   = tn;
    rot_       = rot;
    trans_     = trans;

    qua_ = Eigen::Quaterniond(rot);

    T_.setIdentity();
    T_.block<3, 3>(0, 0) = rot;
    T_.block<3, 1>(0, 3) = trans_;

    T_inv_ = T_.inverse();
}

void ExtrinsicParam::Print()
{
    std::cout << "   --> extrinsic [" << from_name_ << " -> " << to_name_ << "]:\n";
    std::cout << "       R:\n" << rot_ << std::endl;
    std::cout << "       t:\n" << trans_.transpose() << std::endl;
    std::cout << "       qua:\n" << qua_.coeffs().transpose() << std::endl;
    std::cout << "\n";
}

bool ExtrinsicParam::IsMatch(std::string fn, std::string tn)
{
    if (caseInSensStringCompare(from_name_, fn) && caseInSensStringCompare(to_name_, tn)) {
        return true;
    }
    return false;
}

std::string ExtrinsicParam::GetFromName() const
{
    return from_name_;
}
std::string ExtrinsicParam::GetToName() const
{
    return to_name_;
}

bool ExtrinsicParam::IsValid()
{
    Eigen::Matrix3d res = rot_.transpose() - rot_.inverse();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (fabs(res(i, j)) > 0.0001) {
                std::cout << "invalid res=\n" << res << std::endl;
                return false;
            }
        }
    }
    if (from_name_.length() < 1 || to_name_.length() < 1 || strcasecmp(from_name_.c_str(), to_name_.c_str()) == 0) {
        return false;
    }

    return true;
}
Eigen::Quaterniond ExtrinsicParam::GetQua()
{
    return qua_;
}
Eigen::Matrix3d ExtrinsicParam::GetRot()
{
    return rot_;
}
Eigen::Vector3d ExtrinsicParam::GetTrans()
{
    return trans_;
}
Eigen::Vector3d ExtrinsicParam::GetRPY()
{
    return rpy_;
}
Eigen::Matrix4d ExtrinsicParam::GetT()
{
    return T_;
}
Eigen::Matrix4d ExtrinsicParam::GetTInverse()
{
    return T_inv_;
}

float IntrinsicParam::GetFx() const
{
    return intrinsics_(0, 0);
}
float IntrinsicParam::GetFy() const
{
    return intrinsics_(1, 1);
}
float IntrinsicParam::GetCx() const
{
    return intrinsics_(0, 2);
}
float IntrinsicParam::GetCy() const
{
    return intrinsics_(1, 2);
}

std::string IntrinsicParam::GetCameraId() const
{
    return camera_id_;
}
int IntrinsicParam::GetWidth() const
{
    return width_;
}
int IntrinsicParam::GetHeight() const
{
    return height_;
}

// multiply s
bool IntrinsicParam::ScaleTo(float s, IntrinsicParam& new_intrisic)
{
    if (s <= 0) {
        return false;
    }
    new_intrisic.intrinsics_ = this->intrinsics_ * s;
    new_intrisic.width_      = this->width_ * s;
    new_intrisic.height_     = this->height_ * s;

    return true;
}

Eigen::Matrix3d IntrinsicParam::GetIntrinsic()
{
    return intrinsics_;
}

void IntrinsicParam::Print()
{
    std::cout << "\n----camera : " << camera_id_ << "----\n";
    std::cout << "intrinsic:\n" << intrinsics_ << std::endl;
    std::cout << "width=" << width_ << ",height=" << height_ << std::endl;
    std::cout << "-----------------------------------------\n\n\n";
}

void ExtrinsicParamManager::Parse(std::string config_file)
{
    // if file exist
    if (!DoesFileExist(config_file)) {
        std::cout << "not found extrinsic param file ! " << config_file << std::endl;
        return;
    }

    // load
    YAML::Node config = YAML::LoadFile(config_file);

    // if node ok
    if (config.IsNull()) {
        std::cout << "invalid extrinsic file !  " << config_file << std::endl;
        return;
    }

    //--lidar to imu---//
    for (const auto& kv : config) {
        std::string name_pair = kv.first.as<std::string>();
        int         idx       = name_pair.find("_to_");
        if (idx == name_pair.npos || idx < 1 || idx + 4 >= name_pair.length()) {
            std::cout << "not a proper extrinsic tag.\n";

            if (name_pair.find("camera_") == 0) {
                std::cout << "add initrinsic for camera:" << name_pair << std::endl;
                IntrinsicParam intrinc;
                intrinc.camera_id_                       = name_pair;
                const std::vector<double> intrinsics_vec = config[name_pair]["intrinsics"].as<std::vector<double>>();
                intrinc.intrinsics_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(intrinsics_vec.data());

                intrinc.distortion_coeffs_ = config[name_pair]["distortion_coeffs"].as<std::vector<double>>();

                std::vector<int> resolutions = config[name_pair]["resolution"].as<std::vector<int>>();
                if (resolutions.size() == 2) {
                    intrinc.width_  = resolutions[0];
                    intrinc.height_ = resolutions[1];
                }
                intrinc.topic_               = config[name_pair]["cam_topic"].as<std::string>();
                intrinsic_params_[name_pair] = intrinc;
            }
            continue;
        }
        std::string fn = name_pair.substr(0, idx);
        std::string tn = name_pair.substr(idx + 4);
        // std::cout<<"fn="<<fn<<",tn="<<tn<<std::endl;

        Eigen::Matrix3d R;
        Eigen::Vector3d trans;
        // R
        const std::vector<double> R_vec = config[name_pair]["R"].as<std::vector<double>>();
        R                               = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_vec.data());
        // std::cout<<"get R:\n"<<R<<std::endl;

        // t
        std::vector<double> t_vec = config[name_pair]["T"].as<std::vector<double>>();
        trans                     = Eigen::Vector3d(t_vec.data());
        // std::cout<<"get t:\n"<<trans.transpose()<<std::endl;

        ExtrinsicParam ep(fn, tn, R, trans);
        this->AddExtrinsicParam(ep);
    }
}

std::string ExtrinsicParamManager::AddExtrinsicParam(ExtrinsicParam ep)
{
    std::lock_guard<std::mutex> lck(mtx_);
    // std::cout << "AddExtrinsicParam~\n";
    std::string code = GenerateExtrinsicPairCode(ep.GetFromName(), ep.GetToName());
    auto        iter = extrinc_params_.find(code);
    if (iter != extrinc_params_.end()) {
        std::cout << "duplicate extrinsic parameter between: " << ep.GetFromName() << " -> " << ep.GetToName()
                  << std::endl;
        return "";
    }

    if (!ep.IsValid()) {
        std::cout << "invalid extrinsic parameter between: " << ep.GetFromName() << " -> " << ep.GetToName()
                  << std::endl;
        return "";
    }

    this->extrinc_params_[code] = std::move(ep);

    return code;
}
bool ExtrinsicParamManager::GetExtrinsicParamByName(std::string fn, std::string tn, ExtrinsicParam& res)
{
    std::string code = GenerateExtrinsicPairCode(fn, tn);
    return GetExtrinsicParamByCode(code, res);
}
bool ExtrinsicParamManager::GetExtrinsicParamByCode(std::string code, ExtrinsicParam& param)
{
    std::lock_guard<std::mutex> lck(mtx_);
    auto                        iter = extrinc_params_.find(code);
    if (iter != extrinc_params_.end()) {
        param = iter->second;
        return true;
    }
    return false;
}

bool ExtrinsicParamManager::FindCameraIntrinsicByCameraId(std::string camera_id, IntrinsicParam& camera_intrinsic)
{
    auto iter = intrinsic_params_.find(camera_id);
    if (iter == intrinsic_params_.end()) {
        return false;
    }

    camera_intrinsic = iter->second;
    return true;
}

bool ExtrinsicParamManager::DeleteByCode(std::string code)
{
    std::lock_guard<std::mutex> lck(mtx_);
    auto                        iter = extrinc_params_.find(code);
    if (iter != extrinc_params_.end()) {
        extrinc_params_.erase(code);
    }
}
void ExtrinsicParamManager::Clear()
{
    std::lock_guard<std::mutex> lck(mtx_);
    extrinc_params_.clear();
}

std::string ExtrinsicParamManager::GenerateExtrinsicPairCode(std::string fn, std::string tn)
{
    return fn + "_pxto_" + tn;
}

void ExtrinsicParamManager::Print()
{
    std::cout << "\n\n-------------->> Print ExtrinsicParamManager size=" << extrinc_params_.size() << "------------\n";
    for (auto iter = extrinc_params_.begin(); iter != extrinc_params_.end(); iter++) {
        iter->second.Print();
    }
}