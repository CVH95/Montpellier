#include <iostream>
#include <rw/math.hpp> // Pi, Deg2Rad
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/EAA.hpp>

using namespace std;
using namespace rw::math;

int main(int argc, char** argv) {
	cout << " --- Program started --- " << endl << endl;

	cout << "Joint configuration of size 6:" << endl;
	Q q = Q(6,1,2,3,4,5,6);
	cout << q << endl << endl;

	cout << "Transform3D with a position vector at zero and a identity rotation:" << endl;
	Transform3D<> T = Transform3D<>();
	cout << T << endl << endl;

	cout << "Transform3D constructed by position vector and PRY rotation (or Rotation3D) and get its position and rotation:" << endl;
	Vector3D<> V1 = Vector3D<>(42, 84, 126);
	cout << V1 << endl;
	RPY<> R1 = RPY<>(Pi/2, 15*Deg2Rad, Pi);
	cout << R1 << endl;
	cout << R1.toRotation3D() << endl;
	Transform3D<> T1 = Transform3D<>(V1, R1.toRotation3D());
	cout << T1 << endl;
	cout << T1.P() << endl;
	cout << T1.R() << endl << endl;

	cout << "Equivalent angle axis constructed by a direction vector:" << endl;
	Vector3D<> V = Vector3D<>(42, 84, 126);
	V /= V.norm2();
	EAA<> eaa = EAA<>(V,45*Deg2Rad);
	cout << eaa << endl;
	cout << eaa.angle() << " = " << 45*Deg2Rad << endl;
	cout << eaa.axis() << " = " << V << endl << endl;

	cout << " --- Program ended ---" << endl;
	return 0;
}
