#include <iostream>
#include <fstream>
#include <cstring>
#include <vector>

#include <math.h>
#include <stdlib.h>

#include "Skeleton.h"

Joint::Joint()
    : id(0)
    , parent_id(-1)
    , position(Vector3::Zero())
    , rotation(Matrix_4x4::Id()) {}

Joint::Joint(int id, int parent_id)
    : id(id)
    , parent_id(parent_id) {}

Skeleton::Skeleton()
    : m_joints(NULL)
    , m_num_joints(0) {}

Skeleton::~Skeleton() {
    delete[] m_joints;
}

int Skeleton::NumJoints() {
    return m_num_joints;
}

Joint Skeleton::GetJoint(int i) {
    return m_joints[i];
}

void Skeleton::SetJoint(int i, Joint j) {
    m_joints[i] = j;
}

/*
** TODO: Implement. This method must return the global transform of a joint
*/
Matrix_4x4 Skeleton::JointTransform(int i) {

    Joint current_joint = this->GetJoint(i);
    Matrix_4x4 HomoTran_Joint=current_joint.rotation;
    HomoTran_Joint.xw = current_joint.position.x;
    HomoTran_Joint.yw = current_joint.position.y;
    HomoTran_Joint.zw = current_joint.position.z;

    int parent_id = current_joint.parent_id;

    Joint parent_joint = this->GetJoint(parent_id);

    while(parent_id!=-1){
        Matrix_4x4 HomoTran_Parent = parent_joint.rotation;
        HomoTran_Parent.xw = parent_joint.position.x;
        HomoTran_Parent.yw = parent_joint.position.y;
        HomoTran_Parent.zw = parent_joint.position.z;
        
        HomoTran_Joint = HomoTran_Parent*HomoTran_Joint;

        parent_id = parent_joint.parent_id;
        parent_joint = this->GetJoint(parent_id);
    }
    return HomoTran_Joint;
}


Skeleton* Skeleton::Copy() {

    Skeleton* copy = new Skeleton();
    copy->m_num_joints = m_num_joints;
    copy->m_joints = new Joint[copy->m_num_joints];

    for(int i = 0; i < copy->m_num_joints; i++) {
        copy->SetJoint(i, m_joints[i]);
    }

    return copy;
}
