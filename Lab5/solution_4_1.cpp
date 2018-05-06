#include <iostream>
#include <vector>
#include <algorithm>
#include <rw/math.hpp>

// This function is the solution to Programming Exercise 3.4
rw::math::Transform3D<> forwardKinematics(const std::vector<rw::math::Transform3D<>>& trefs,
					  const unsigned int idx, const rw::math::Q& q) {
  if(trefs.size() != q.size()) {
    RW_THROW("The number of local transformations must be equal to the length of the configuration vector");
  }

  // Initialize empty transform to contain the desired transformation
  rw::math::Transform3D<> baseTi;

  // Fill baseTi
  for(unsigned int i = 0; i < idx; ++i) {
    // Calculate Tz as per Equation 3.18
    rw::math::Transform3D<> Tz(rw::math::RPY<>(q[i], 0, 0).toRotation3D());
    // Calculate (i-1)Ti (Equation after 3.21 in the notes)
    rw::math::Transform3D<> Ti = trefs[i]*Tz;
    // This line implements Equation 3.21
    baseTi = baseTi*Ti;
  }

  return baseTi;
}

// In order to calculate the Jacobian for the inverse kinematics we need the the
// transformation baseTi for all joints in the robot and also the transformation
// baseTtool
rw::math::Jacobian calculateJacobian(const std::vector<rw::math::Transform3D<>>& tvec,
                                     const rw::math::Transform3D<>& baseTtool) {
    // Calculate A and B
    std::vector<rw::math::Vector3D<>> A, B;
    for(const rw::math::Transform3D<>& T : tvec) {
        // Create a column of the position part of the Jacobian. This line is
        // Equation 4.4 from the Robotics notes
        A.push_back(rw::math::cross(T.R().getCol(2), baseTtool.P() - T.P()));
	// Create a column of the rotation part of the Jacobian. This line is
	// Equation 4.6 from the Robotics notes.
	B.push_back(T.R().getCol(2));
    }

    // Fill the Jacobian matrix with A and B
    rw::math::Jacobian J = rw::math::Jacobian::zero(6, tvec.size());
    for(unsigned int i = 0; i < tvec.size(); ++i) {
        // The addPosition function fills column i with the vector A[i] to the position part of the
        // Jacobian starting at row 0 of the position part. Our Jacobian is 6x3 with a 3x3 position part
        // and a 3x3 rotation part. The addRotation functions the same way just with the rotation part
        // of the Jacobian.
        J.addPosition(A[i], 0, i);
        J.addRotation(B[i], 0, i);
    }

    return J;
}

int main() {
    // Test is example 4.2.1 from the book
    // Initialize configuration
    rw::math::Q q(3, 0, -rw::math::Pi/6, rw::math::Pi/6);

    // Create Trefs
    int L = 3, a2 = 2, a3 = 2;
    std::vector<rw::math::Transform3D<>> trefs;

    // Base -> 1
    rw::math::Vector3D<> v1(0,0,L);
    rw::math::RPY<> r1(0,0,0);
    trefs.push_back(rw::math::Transform3D<>(v1, r1.toRotation3D()));

    // 1 -> 2
    rw::math::Vector3D<> v2(0,0,0);
    rw::math::RPY<> r2(0,0,rw::math::Pi/2);
    trefs.push_back(rw::math::Transform3D<>(v2, r2.toRotation3D()));

    // 2 -> 3
    rw::math::Vector3D<> v3(0,a2,0);
    rw::math::RPY<> r3(0,0,0);
    trefs.push_back(rw::math::Transform3D<>(v3, r3.toRotation3D()));

    // Calculate transforms
    // We need all the baseTi's to have access to baseZi and basePi in
    // Equations 4.4 and 4.6 from the notes.
    std::vector<rw::math::Transform3D<>> Tvec;
    for(unsigned int i = 1; i <= trefs.size(); ++i) {
        Tvec.push_back(forwardKinematics(trefs, i, q));
    }

    // Fill in the transformation iTtool
    // 3 -> TCP
    rw::math::Vector3D<> v4(a3,0,0);
    rw::math::RPY<> r4(0,0,0);
    rw::math::Transform3D<> Ttool(v4, r4.toRotation3D());
    rw::math::Transform3D<> baseTtcp = Tvec.back()*Ttool;

    // Create Jacobian
    rw::math::Jacobian J = calculateJacobian(Tvec, baseTtcp);

    std::cout << J << std::endl;
    return 0;
}
