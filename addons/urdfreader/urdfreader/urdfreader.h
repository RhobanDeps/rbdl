#ifndef RBDL_URDFREADER_H
#define RBDL_URDFREADER_H

#include <rbdl/rbdl_config.h>

namespace RigidBodyDynamics {

struct Model;

namespace Addons {
	RBDL_DLLAPI bool URDFReadFromFile (const char* filename, Model* model, bool verbose = false, 
            Eigen::MatrixXd* inertiaData = NULL, std::map<std::string, size_t>* inertiaName = NULL, 
            bool setInertia = false,
            Eigen::MatrixXd* geometryData = NULL, std::map<std::string, size_t>* geometryName = NULL, 
            bool setGeometry = false);
	RBDL_DLLAPI bool URDFReadFromString (const char* model_xml_string, Model* model, bool verbose = false,
            Eigen::MatrixXd* inertiaData = NULL, std::map<std::string, size_t>* inertiaName = NULL, 
            bool setInertia = false,
            Eigen::MatrixXd* geometryData = NULL, std::map<std::string, size_t>* geometryName = NULL, 
            bool setGeometry = false);
}

}

/* _RBDL_URDFREADER_H */
#endif
