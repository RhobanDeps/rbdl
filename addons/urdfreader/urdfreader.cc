#include <rbdl/rbdl.h>

#include "urdfreader.h"

#include <assert.h>
#include <iostream>
#include <fstream>
#include <map>
#include <stack>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

using namespace std;

namespace RigidBodyDynamics {

namespace Addons {

using namespace Math;

typedef boost::shared_ptr<urdf::Link> LinkPtr;
typedef boost::shared_ptr<urdf::Joint> JointPtr;
typedef boost::shared_ptr<urdf::ModelInterface> ModelPtr;

typedef vector<LinkPtr> URDFLinkVector;
typedef vector<JointPtr> URDFJointVector;
typedef map<string, LinkPtr > URDFLinkMap;
typedef map<string, JointPtr > URDFJointMap;

bool construct_model (Model* rbdl_model, ModelPtr urdf_model, bool verbose, 
    Eigen::MatrixXd* inertiaData = NULL, std::map<std::string, size_t>* inertiaName = NULL,
    bool setInertia = false,
    Eigen::MatrixXd* geometryData = NULL, std::map<std::string, size_t>* geometryName = NULL,
    bool setGeometry = false) {
        if (inertiaData != NULL && inertiaName == NULL) {
            throw std::logic_error("RBDL no inertia name");
        }
        if (inertiaData != NULL && setInertia == false) {
            *inertiaData = Eigen::MatrixXd();
            inertiaName->clear();
        }

	boost::shared_ptr<urdf::Link> urdf_root_link;

	URDFLinkMap link_map;
	link_map = urdf_model->links_;

	URDFJointMap joint_map;
	joint_map = urdf_model->joints_;

	vector<string> joint_names;

	stack<LinkPtr > link_stack;
	stack<int> joint_index_stack;

	// add the bodies in a depth-first order of the model tree
	link_stack.push (link_map[(urdf_model->getRoot()->name)]);

	// add the root body
	const boost::shared_ptr<const urdf::Link>& root = urdf_model->getRoot ();
	Vector3d root_inertial_rpy;
	Vector3d root_inertial_position;
	Matrix3d root_inertial_inertia;
	double root_inertial_mass;

	if (root->inertial) {
		root_inertial_mass = root->inertial->mass;

		root_inertial_position.set (
				root->inertial->origin.position.x,
				root->inertial->origin.position.y,
				root->inertial->origin.position.z);

		root_inertial_inertia(0,0) = root->inertial->ixx;
		root_inertial_inertia(0,1) = root->inertial->ixy;
		root_inertial_inertia(0,2) = root->inertial->ixz;

		root_inertial_inertia(1,0) = root->inertial->ixy;
		root_inertial_inertia(1,1) = root->inertial->iyy;
		root_inertial_inertia(1,2) = root->inertial->iyz;

		root_inertial_inertia(2,0) = root->inertial->ixz;
		root_inertial_inertia(2,1) = root->inertial->iyz;
		root_inertial_inertia(2,2) = root->inertial->izz;

                if (inertiaData != NULL && setInertia == false) {
                    size_t row = inertiaData->rows();
                    inertiaData->conservativeResize(row+1, 10);
                    inertiaData->operator()(row, 0) = root->inertial->mass;
                    inertiaData->operator()(row, 1) = root->inertial->origin.position.x;
                    inertiaData->operator()(row, 2) = root->inertial->origin.position.y;
                    inertiaData->operator()(row, 3) = root->inertial->origin.position.z;
                    inertiaData->operator()(row, 4) = root->inertial->ixx;
                    inertiaData->operator()(row, 5) = root->inertial->ixy;
                    inertiaData->operator()(row, 6) = root->inertial->ixz;
                    inertiaData->operator()(row, 7) = root->inertial->iyy;
                    inertiaData->operator()(row, 8) = root->inertial->iyz;
                    inertiaData->operator()(row, 9) = root->inertial->izz;
                    inertiaName->operator[](root->name) = row;
                }
                if (inertiaData != NULL && setInertia == true) {
                    if (inertiaName->count(root->name) == 0 || inertiaData->rows() < inertiaName->at(root->name)) {
                        throw std::logic_error("RBDL inertiaData root row not found: " + root->name);
                    }
                    size_t row = inertiaName->at(root->name);
                    root_inertial_mass = inertiaData->operator()(row, 0);
                    root_inertial_position.set (
                        inertiaData->operator()(row, 1),
                        inertiaData->operator()(row, 2),
                        inertiaData->operator()(row, 3));
                    root_inertial_inertia(0,0) = inertiaData->operator()(row, 4);
                    root_inertial_inertia(0,1) = inertiaData->operator()(row, 5);
                    root_inertial_inertia(0,2) = inertiaData->operator()(row, 6);
                    root_inertial_inertia(1,0) = inertiaData->operator()(row, 5);
                    root_inertial_inertia(1,1) = inertiaData->operator()(row, 7);
                    root_inertial_inertia(1,2) = inertiaData->operator()(row, 8);
                    root_inertial_inertia(2,0) = inertiaData->operator()(row, 6);
                    root_inertial_inertia(2,1) = inertiaData->operator()(row, 8);
                    root_inertial_inertia(2,2) = inertiaData->operator()(row, 9);
                }

		root->inertial->origin.rotation.getRPY (root_inertial_rpy[0], root_inertial_rpy[1], root_inertial_rpy[2]);

		Body root_link = Body (root_inertial_mass,
				root_inertial_position,
				root_inertial_inertia);
	        Joint root_joint = Joint ( JointTypeFixed , "root");
                /*
		Joint root_joint = Joint (
				SpatialVector (0., 0., 0., 1., 0., 0.),
				SpatialVector (0., 0., 0., 0., 1., 0.),
				SpatialVector (0., 0., 0., 0., 0., 1.),
				SpatialVector (1., 0., 0., 0., 0., 0.),
				SpatialVector (0., 1., 0., 0., 0., 0.),
				SpatialVector (0., 0., 1., 0., 0., 0.));
                */

		SpatialTransform root_joint_frame = SpatialTransform ();

		if (verbose) {
			cout << "+ Adding Root Body " << endl;
			cout << "  joint frame: " << root_joint_frame << endl;
			cout << "  joint dofs : " << root_joint.mDoFCount << endl;
			for (unsigned int j = 0; j < root_joint.mDoFCount; j++) {
				cout << "    " << j << ": " << root_joint.mJointAxes[j].transpose() << endl;
			}
			cout << "  body inertia: " << endl << root_link.mInertia << endl;
			cout << "  body mass   : " << root_link.mMass << endl;
			cout << "  body name   : " << root->name << endl;
		}

		rbdl_model->AppendBody(root_joint_frame,
				root_joint,
				root_link,
				root->name);
	}

	if (link_stack.top()->child_joints.size() > 0) {
		joint_index_stack.push(0);
	}

	while (link_stack.size() > 0) {
		LinkPtr cur_link = link_stack.top();
		unsigned int joint_idx = joint_index_stack.top();

		if (joint_idx < cur_link->child_joints.size()) {
			JointPtr cur_joint = cur_link->child_joints[joint_idx];

			// increment joint index
			joint_index_stack.pop();
			joint_index_stack.push (joint_idx + 1);

			link_stack.push (link_map[cur_joint->child_link_name]);
			joint_index_stack.push(0);

			if (verbose) {
				for (unsigned int i = 1; i < joint_index_stack.size() - 1; i++) {
					cout << "  ";
				}
				cout << "joint '" << cur_joint->name << "' child link '" << link_stack.top()->name << "' type = " << cur_joint->type << endl;
			}

			joint_names.push_back(cur_joint->name);
		} else {
			link_stack.pop();
			joint_index_stack.pop();
		}
	}

	unsigned int j;
	for (j = 0; j < joint_names.size(); j++) {
		JointPtr urdf_joint = joint_map[joint_names[j]];
		LinkPtr urdf_parent = link_map[urdf_joint->parent_link_name];
		LinkPtr urdf_child = link_map[urdf_joint->child_link_name];

		// determine where to add the current joint and child body
		unsigned int rbdl_parent_id = 0;

		if (urdf_parent->name != "base_joint" && rbdl_model->mBodies.size() != 1)
			rbdl_parent_id = rbdl_model->GetBodyId (urdf_parent->name.c_str());

		if (rbdl_parent_id == std::numeric_limits<unsigned int>::max())
			cerr << "Error while processing joint '" << urdf_joint->name
				<< "': parent link '" << urdf_parent->name
				<< "' could not be found." << endl;

		//cout << "joint: " << urdf_joint->name << "\tparent = " << urdf_parent->name << " child = " << urdf_child->name << " parent_id = " << rbdl_parent_id << endl;

		// create the joint
		Joint rbdl_joint;
		if (urdf_joint->type == urdf::Joint::REVOLUTE || urdf_joint->type == urdf::Joint::CONTINUOUS) {
			rbdl_joint = Joint (SpatialVector (urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z, 0., 0., 0.), 
                            urdf_joint->name);
		} else if (urdf_joint->type == urdf::Joint::PRISMATIC) {
			rbdl_joint = Joint (SpatialVector (0., 0., 0., urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z),
                                urdf_joint->name);
		} else if (urdf_joint->type == urdf::Joint::FIXED) {
			rbdl_joint = Joint (JointTypeFixed, urdf_joint->name);
		} else if (urdf_joint->type == urdf::Joint::FLOATING) {
			// todo: what order of DoF should be used?
			rbdl_joint = Joint (
					SpatialVector (0., 0., 0., 1., 0., 0.),
					SpatialVector (0., 0., 0., 0., 1., 0.),
					SpatialVector (0., 0., 0., 0., 0., 1.),
					SpatialVector (1., 0., 0., 0., 0., 0.),
					SpatialVector (0., 1., 0., 0., 0., 0.),
					SpatialVector (0., 0., 1., 0., 0., 0.), 
                                        urdf_joint->name);
		} else if (urdf_joint->type == urdf::Joint::PLANAR) {
			// todo: which two directions should be used that are perpendicular
			// to the specified axis?
			cerr << "Error while processing joint '" << urdf_joint->name << "': planar joints not yet supported!" << endl;
			return false;
		}

		// compute the joint transformation
		Vector3d joint_rpy;
		Vector3d joint_translation;
		urdf_joint->parent_to_joint_origin_transform.rotation.getRPY (joint_rpy[0], joint_rpy[1], joint_rpy[2]);
		joint_translation.set (
				urdf_joint->parent_to_joint_origin_transform.position.x,
				urdf_joint->parent_to_joint_origin_transform.position.y,
				urdf_joint->parent_to_joint_origin_transform.position.z
				);

                if (geometryData != NULL && setGeometry == false) {
                    size_t row = geometryData->rows();
                    geometryData->conservativeResize(row+1, 6);
                    geometryData->operator()(row, 0) = joint_rpy(0);
                    geometryData->operator()(row, 1) = joint_rpy(1);
                    geometryData->operator()(row, 2) = joint_rpy(2);
                    geometryData->operator()(row, 3) = joint_translation(0);
                    geometryData->operator()(row, 4) = joint_translation(1);
                    geometryData->operator()(row, 5) = joint_translation(2);
                    geometryName->operator[](urdf_child->name) = row;
                }
                if (geometryData != NULL && setGeometry == true) {
                    if (
                        geometryName->count(urdf_child->name) == 0 || 
                        geometryData->rows() < geometryName->at(urdf_child->name)
                    ) {
                        throw std::logic_error("RBDL geometryData row not found: " + urdf_child->name);
                    }
                    size_t row = geometryName->at(urdf_child->name);
                    joint_rpy(0) = inertiaData->operator()(row, 0);
                    joint_rpy(1) = inertiaData->operator()(row, 1);
                    joint_rpy(2) = inertiaData->operator()(row, 2);
                    joint_translation(0) = inertiaData->operator()(row, 3);
                    joint_translation(1) = inertiaData->operator()(row, 4);
                    joint_translation(2) = inertiaData->operator()(row, 5);
                }

		SpatialTransform rbdl_joint_frame =
					Xrot (joint_rpy[0], Vector3d (1., 0., 0.))
				* Xrot (joint_rpy[1], Vector3d (0., 1., 0.))
				* Xrot (joint_rpy[2], Vector3d (0., 0., 1.))
				* Xtrans (Vector3d (
							joint_translation
							));

		// assemble the body
		Vector3d link_inertial_position;
		Vector3d link_inertial_rpy;
		Matrix3d link_inertial_inertia = Matrix3d::Zero();
		double link_inertial_mass = 0.;

		// but only if we actually have inertial data
		if (urdf_child->inertial) {
			link_inertial_mass = urdf_child->inertial->mass;

			link_inertial_position.set (
					urdf_child->inertial->origin.position.x,
					urdf_child->inertial->origin.position.y,
					urdf_child->inertial->origin.position.z
					);
			urdf_child->inertial->origin.rotation.getRPY (link_inertial_rpy[0], link_inertial_rpy[1], link_inertial_rpy[2]);

			link_inertial_inertia(0,0) = urdf_child->inertial->ixx;
			link_inertial_inertia(0,1) = urdf_child->inertial->ixy;
			link_inertial_inertia(0,2) = urdf_child->inertial->ixz;

			link_inertial_inertia(1,0) = urdf_child->inertial->ixy;
			link_inertial_inertia(1,1) = urdf_child->inertial->iyy;
			link_inertial_inertia(1,2) = urdf_child->inertial->iyz;

			link_inertial_inertia(2,0) = urdf_child->inertial->ixz;
			link_inertial_inertia(2,1) = urdf_child->inertial->iyz;
			link_inertial_inertia(2,2) = urdf_child->inertial->izz;
                
                        if (
                            (urdf_joint->type == urdf::Joint::REVOLUTE || urdf_joint->type == urdf::Joint::CONTINUOUS) &&
                            inertiaData != NULL && setInertia == false
                        ) {
                            size_t row = inertiaData->rows();
                            inertiaData->conservativeResize(row+1, 10);
                            inertiaData->operator()(row, 0) = urdf_child->inertial->mass;
                            inertiaData->operator()(row, 1) = urdf_child->inertial->origin.position.x;
                            inertiaData->operator()(row, 2) = urdf_child->inertial->origin.position.y;
                            inertiaData->operator()(row, 3) = urdf_child->inertial->origin.position.z;
                            inertiaData->operator()(row, 4) = urdf_child->inertial->ixx;
                            inertiaData->operator()(row, 5) = urdf_child->inertial->ixy;
                            inertiaData->operator()(row, 6) = urdf_child->inertial->ixz;
                            inertiaData->operator()(row, 7) = urdf_child->inertial->iyy;
                            inertiaData->operator()(row, 8) = urdf_child->inertial->iyz;
                            inertiaData->operator()(row, 9) = urdf_child->inertial->izz;
                            inertiaName->operator[](urdf_child->name) = row;
                        }
                        if (
                            (urdf_joint->type == urdf::Joint::REVOLUTE || urdf_joint->type == urdf::Joint::CONTINUOUS) &&
                            inertiaData != NULL && setInertia == true
                        ) {
                            if (
                                inertiaName->count(urdf_child->name) == 0 || 
                                inertiaData->rows() < inertiaName->at(urdf_child->name)
                            ) {
                                throw std::logic_error("RBDL inertiaData row not found: " + urdf_child->name);
                            }
                            size_t row = inertiaName->at(urdf_child->name);
                            link_inertial_mass = inertiaData->operator()(row, 0);
                            link_inertial_position.set (
                                inertiaData->operator()(row, 1),
                                inertiaData->operator()(row, 2),
                                inertiaData->operator()(row, 3));
                            link_inertial_inertia(0,0) = inertiaData->operator()(row, 4);
                            link_inertial_inertia(0,1) = inertiaData->operator()(row, 5);
                            link_inertial_inertia(0,2) = inertiaData->operator()(row, 6);
                            link_inertial_inertia(1,0) = inertiaData->operator()(row, 5);
                            link_inertial_inertia(1,1) = inertiaData->operator()(row, 7);
                            link_inertial_inertia(1,2) = inertiaData->operator()(row, 8);
                            link_inertial_inertia(2,0) = inertiaData->operator()(row, 6);
                            link_inertial_inertia(2,1) = inertiaData->operator()(row, 8);
                            link_inertial_inertia(2,2) = inertiaData->operator()(row, 9);
                        }

			if (link_inertial_rpy != Vector3d (0., 0., 0.)) {
				cerr << "Error while processing body '" << urdf_child->name << "': rotation of body frames not yet supported. Please rotate the joint frame instead." << endl;
				return false;
			}
		}

		Body rbdl_body = Body (link_inertial_mass, link_inertial_position, link_inertial_inertia);

		if (verbose) {
			cout << "+ Adding Body " << endl;
			cout << "  parent_id  : " << rbdl_parent_id << endl;
			cout << "  joint frame: " << rbdl_joint_frame << endl;
			cout << "  joint dofs : " << rbdl_joint.mDoFCount << endl;
			for (unsigned int j = 0; j < rbdl_joint.mDoFCount; j++) {
				cout << "    " << j << ": " << rbdl_joint.mJointAxes[j].transpose() << endl;
			}
			cout << "  body inertia: " << endl << rbdl_body.mInertia << endl;
			cout << "  body mass   : " << rbdl_body.mMass << endl;
			cout << "  body name   : " << urdf_child->name << endl;
		}

		if (0 && urdf_joint->type == urdf::Joint::FLOATING) {
			Matrix3d zero_matrix = Matrix3d::Zero();
			Body null_body (0., Vector3d::Zero(3), zero_matrix);
			Joint joint_txtytz(JointTypeTranslationXYZ, urdf_joint->name);
			string trans_body_name = urdf_child->name + "_Translate";
			rbdl_model->AddBody (rbdl_parent_id, rbdl_joint_frame, joint_txtytz, null_body, trans_body_name);

			Joint joint_euler_zyx (JointTypeEulerXYZ, urdf_joint->name);
			rbdl_model->AppendBody (SpatialTransform(), joint_euler_zyx, rbdl_body, urdf_child->name);
		} else {
			rbdl_model->AddBody (rbdl_parent_id, rbdl_joint_frame, rbdl_joint, rbdl_body, urdf_child->name);
		}
	}

	return true;
}

RBDL_DLLAPI bool URDFReadFromFile (const char* filename, Model* model, bool verbose,
    Eigen::MatrixXd* inertiaData, std::map<std::string, size_t>* inertiaName, bool setInertia,
    Eigen::MatrixXd* geometryData, std::map<std::string, size_t>* geometryName, bool setGeometry) {
	ifstream model_file (filename);
	if (!model_file) {
		cerr << "Error opening file '" << filename << "'." << endl;
		abort();
	}
	
	// reserve memory for the contents of the file
	string model_xml_string;
	model_file.seekg(0, std::ios::end);
	model_xml_string.reserve(model_file.tellg());
	model_file.seekg(0, std::ios::beg);
	model_xml_string.assign((std::istreambuf_iterator<char>(model_file)), std::istreambuf_iterator<char>());

	model_file.close();

	return URDFReadFromString (model_xml_string.c_str(), model, verbose, 
            inertiaData, inertiaName, setInertia,
            geometryData, geometryName, setGeometry);
}

RBDL_DLLAPI bool URDFReadFromString (const char* model_xml_string, Model* model, bool verbose,
    Eigen::MatrixXd* inertiaData, std::map<std::string, size_t>* inertiaName, bool setInertia,
    Eigen::MatrixXd* geometryData, std::map<std::string, size_t>* geometryName, bool setGeometry) {
	assert (model);

	boost::shared_ptr<urdf::ModelInterface> urdf_model = urdf::parseURDF (model_xml_string);
 
	if (!construct_model (model, urdf_model, verbose, 
                inertiaData, inertiaName, setInertia,
                geometryData, geometryName, setGeometry)
        ) {
		cerr << "Error constructing model from urdf file." << endl;
		return false;
	}

	model->gravity.set (0., 0., -9.81);

	return true;
}

}

}
