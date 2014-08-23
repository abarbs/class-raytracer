#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE RayTracer
#include <boost/test/unit_test.hpp>
#include <boost/format.hpp>
#include <Eigen/Dense>

#define TEST_QC_DIRECTLY

#include <stdexcept>
#include <iostream>
#include <sstream>
#include <cmath>

#include "SceneDescription.hpp"
#include "SceneObject.hpp"
#include "Light.hpp"
#include "Material.hpp"
#include "Ray.hpp"
#include "Camera.hpp"
#include "QuadricCollection.hpp"
#include "Box.hpp"
#include "Ellipsoid.hpp"
#include "Sphere.hpp"
#include "Cylinder.hpp"

void parseModelFile(std::istream& input, SceneDescription& scene);

template <typename Enumeration>
auto as_integer(Enumeration const value)
    -> typename std::underlying_type<Enumeration>::type
{
    return static_cast<typename std::underlying_type<Enumeration>::type>(value);
}

BOOST_AUTO_TEST_SUITE(ParseInput)

BOOST_AUTO_TEST_CASE(ParseLight) {
    //TODO: randomly generate lights using boost::regex_replace
    SceneDescription scene;
    std::string inputStr = "light ID 0 pnt 100 100 100 -100 -100 -100 " 
	"20 20 20 0.5 0.2 0.9  1 shadow_on\n"
	"light ID 1 pnt 10 10 10 -10 -10 -10 20 20 20 0.5 0.2 0.9 2 shadow_off\n";
    std::stringstream input(inputStr);
    parseModelFile(input, scene);
    const std::vector<std::unique_ptr<Light> > &lights = scene.getLights();
    BOOST_CHECK_EQUAL(lights.size(), 2);
    
    const std::unique_ptr<Light> &l1 = lights[0];
    BOOST_CHECK_EQUAL(as_integer(l1->m_type), as_integer(Light::LightType::POINT));
    BOOST_CHECK_EQUAL(l1->m_pos,  Eigen::Vector4d(100, 100, 100, 1));
    BOOST_CHECK(l1->m_dir.isApprox(Eigen::Vector4d(-100, -100, -100, 0).normalized()));
    BOOST_CHECK_EQUAL(l1->m_rgb, Eigen::Vector4d(20, 20, 20, 0));
    BOOST_CHECK_EQUAL(l1->m_shadowOn, true);
}

BOOST_AUTO_TEST_CASE(ParseMaterial) {
    std::string inputStr = "mat ID 0 0.1 0.2 0.3 4 5 6 7 8 9 10\n"
	"mat ID 1 0.9 0.8 0.7 6 5 4 3 2 1 0\n";
    std::stringstream input(inputStr);
    SceneDescription scene;
    parseModelFile(input, scene);

    BOOST_CHECK_NO_THROW(scene.getMaterial(0));
    BOOST_CHECK_NO_THROW(scene.getMaterial(1));

    Material m = scene.getMaterial(1);
    BOOST_CHECK_EQUAL(m.id, 1);
    BOOST_CHECK_EQUAL(m.rgb.isApprox(Eigen::Vector4d(0.9, 0.8, 0.7, 0)), true);
    BOOST_CHECK_EQUAL(m.Ka, 6);
    BOOST_CHECK_EQUAL(m.Kd, 5);
    BOOST_CHECK_EQUAL(m.Ks, 4);
    BOOST_CHECK_EQUAL(m.ns, 3);
    BOOST_CHECK_EQUAL(m.Kt, 2);
    BOOST_CHECK_EQUAL(m.Kr, 1);
    BOOST_CHECK_EQUAL(m.Irefract, 0);

}

BOOST_AUTO_TEST_CASE(ParseCamera) {
    const std::string wrongInput = "camera orthographic 5 -5 5 0 0 0 0 0 1 5  0.1 20\n"
	"viewport  400 400\n"
	"light ID 0 inf 5 -1.25 12 0 0 -1 0.35 0.35 0.35 1 0 0.00001 0 shadow_on\n"
	"light ID 1 pnt 5 7 3-1 -1 0 1 0.0 0 1 0 0.0025 5 shadow_off";

    const std::string correctInput = "camera orthographic 5 -5 5 0 0 0 0 0 1 5  0.1 20\n"
	"resolution  0.025 0.025\n"
	"viewport  400 400\n";

    BOOST_CHECK_THROW(Camera::fromString(wrongInput), std::invalid_argument);    
    Camera c = Camera::fromString(correctInput);
    BOOST_CHECK_EQUAL(true, Camera::CameraType::ORTHOGRAPHIC == c.type);
    BOOST_CHECK_EQUAL(c.xRes, 0.025);
    BOOST_CHECK_EQUAL(c.yRes, 0.025);
    BOOST_CHECK_EQUAL(c.viewportWidth, 400);
    BOOST_CHECK_EQUAL(c.viewportHeight, 400);
    BOOST_CHECK_EQUAL(true, Eigen::Vector4d(5, -5, 5, 1).isApprox(c.cop));
    BOOST_CHECK_EQUAL(true, Eigen::Vector3d(0, 0, 0).isApprox(c.look));
    BOOST_CHECK_EQUAL(true, Eigen::Vector3d(0, 0, 1).isApprox(c.up));
    BOOST_CHECK_EQUAL(true, Eigen::Vector4d(0.70710678, 0.70710678, 0, 0).isApprox(c.u, 16));
    BOOST_CHECK_EQUAL(true, Eigen::Vector4d(-0.40824829, 0.40824829, 0.81649658, 0).isApprox(c.v, 8));

}



BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(RayTest)


BOOST_AUTO_TEST_CASE(VectorToLight) {
    int id = 0;
    Light::LightType type = Light::LightType::POINT;
    Eigen::Vector4d  pos(3, 3, 3, 1), dir(-1, -1, -1, 0);
    Eigen::Vector4d rgb(100, 100, 100, 0);
    Eigen::Vector4d radialAttenuation(0.5, 0.5, 0.5, 0);
    float angularAttenuation = 1.5;
    bool shadowOn = true;
    Light l(id, type, pos, dir, rgb, radialAttenuation,
	    angularAttenuation, shadowOn);

    Eigen::Vector4d worldPoint(0, 0, 0, 1);
    BOOST_CHECK_EQUAL(l.getVectorToLight(worldPoint).isApprox(
		      Eigen::Vector4d(1, 1, 1, 0).normalized()), true);

    worldPoint << 5,5,5,1;
    BOOST_CHECK_EQUAL(l.getVectorToLight(worldPoint).isApprox(
		      Eigen::Vector4d(-1,-1,-1, 0).normalized()), true);

    type = Light::LightType::AMBIENT;
    l = Light(id, type, pos, dir, rgb, radialAttenuation,
	      angularAttenuation, shadowOn);

    BOOST_CHECK_THROW(l.getVectorToLight(worldPoint), std::runtime_error);
}


BOOST_AUTO_TEST_SUITE_END()


BOOST_AUTO_TEST_SUITE(QuadricCollectionTests)

#ifdef TEST_QC_DIRECTLY
BOOST_AUTO_TEST_CASE(addQuadric) {
    int id = 4, matId = 5;
    QuadricCollection qc(Eigen::Vector4d(0, 0, 0, 1), 4, 5);
    BOOST_CHECK_EQUAL(qc.id, id);
    BOOST_CHECK_EQUAL(qc.matId, matId);
    BOOST_CHECK(qc.center.isApprox(Eigen::Vector4d(0, 0, 0, 1)));
    Eigen::Matrix4d firstQuadric, secondQuadric;
    firstQuadric << 1, 0, 0, -3,
	0, 1, 0, -3,
	0, 0, 1, -3,
	-3, -3, -3, 26;
    secondQuadric << 0, 0, 0, 0.5,
	0, 0, 0, 0.5,
	0, 0, 0, 0.5,
	0.5, 0.5, 0.5, -1;
    qc.addQuadric(firstQuadric);
    qc.addQuadric(secondQuadric);
    BOOST_CHECK(qc.quadrics[0].isApprox(firstQuadric));
    BOOST_CHECK(qc.quadrics[1].isApprox(secondQuadric));
}
#endif

BOOST_AUTO_TEST_CASE(distToIntersectSphere) {
    QuadricCollection qc(Eigen::Vector4d(5,5,5,1), 0, 4);
    /* Make a sphere of radius 2 centered at (5, 5, 5),
       shoot a ray from origin towards (5, 5, 3), check
       distance is norm of 5,5,3 */
    Eigen::Matrix4d sphere;
    sphere << 0.25, 0, 0, 0,
	0, 0.25, 0, 0,
	0, 0, 0.25, 0,
	0, 0, 0, -1;
    Ray r(Eigen::Vector4d(0, 0, 0, 1), Eigen::Vector4d(5, 5, 3, 0).normalized());
    qc.addQuadric(sphere);
    BOOST_CHECK(std::abs(qc.distToIntersection(r)- sqrt(59)) < EPSILON);

    /* Make a ray originating on top of sphere heading through south pole, check
       that distance is approximately equal to radius */
    Ray rp(Eigen::Vector4d(5,5,7,1), Eigen::Vector4d(0, 0, -1, 0));
    double dist = qc.distToIntersection(rp);
    BOOST_CHECK_MESSAGE(std::abs(dist-4) < EPSILON, 
		    "Got distance to intersection as " << dist);
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(testBox)

BOOST_AUTO_TEST_CASE(testBoxOnSurface) {
    Eigen::Vector4d center(5, 5, 5, 1);
    Eigen::Vector4d u(1, 0, 0, 0);
    Eigen::Vector4d v(0, 1, 0, 0);
    Eigen::Vector4d w(0, 0, 1, 0);
    int id = 4, matId = 6;
    double uWidth = 2, vWidth = 4, wWidth = 3;

    Box b(center, u, v, w, id, matId, uWidth, vWidth, wWidth);
    BOOST_CHECK_EQUAL(b.id, id);
    BOOST_CHECK_EQUAL(b.matId, matId);
    
    BOOST_CHECK(b.onSurface(Eigen::Vector4d(4, 3, 5 - 3/2.0, 1)));
    BOOST_CHECK(b.onSurface(Eigen::Vector4d(6, 7, 5 + 3/2.0, 1)));
    BOOST_CHECK(!b.onSurface(Eigen::Vector4d(7, 7, 5 + 3/2.0, 1)));

    /* Try a rotated box */
    u = Eigen::Vector4d(0, 0, 1, 0);
    v = Eigen::Vector4d(0, 1, 0, 0);
    w = Eigen::Vector4d(-1, 0, 0, 0);

    Box rotatedBox(center, u, v, w, id, matId, uWidth, vWidth, wWidth);
    BOOST_CHECK(rotatedBox.onSurface(Eigen::Vector4d(5 + 3/2.0, 7, 6, 1)));
    
    /* Perform the same checks on a Box constructed from a string */
    std::string boxString = 
	"\nobj box ID 7 mat ID 6 5 5 5 1 0 0 0 1 0 0 0 1 2 4 3\n";
    Box strBox = Box::fromString(boxString);
    BOOST_CHECK_EQUAL(strBox.id, 7);
    BOOST_CHECK_EQUAL(strBox.matId, 6);
    
    BOOST_CHECK(strBox.onSurface(Eigen::Vector4d(4, 3, 5 - 3/2.0, 1)));
    BOOST_CHECK(strBox.onSurface(Eigen::Vector4d(6, 7, 5 + 3/2.0, 1)));
    BOOST_CHECK(!strBox.onSurface(Eigen::Vector4d(7, 7, 5 + 3/2.0, 1)));

    std::string rotatedBoxString = 
	"obj box ID 4 mat ID 6 5 5 5 0 0 1 0 1 0 -1 0 0 2 4 3";
    Box rotatedStrBox = Box::fromString(rotatedBoxString);
    BOOST_CHECK(rotatedStrBox.onSurface(Eigen::Vector4d(5 + 3/2.0, 7, 6, 1)));

}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(testEllipsoid)

BOOST_AUTO_TEST_CASE(parseEllipsoid) {
    std::string elStr = "\nobj ellipsoid ID 2 mat ID 3 4 5 6 2.0 3.0 1.0\n";

    Ellipsoid el = Ellipsoid::fromString(elStr);
    BOOST_CHECK_EQUAL(el.id, 2);
    BOOST_CHECK_EQUAL(el.matId, 3);
    BOOST_CHECK(el.center.isApprox(Eigen::Vector4d(4, 5, 6, 1)));
    BOOST_CHECK(el.onSurface(Eigen::Vector4d(6, 5, 6, 1)));
    BOOST_CHECK(el.onSurface(Eigen::Vector4d(4, 8, 6, 1)));
    BOOST_CHECK(el.onSurface(Eigen::Vector4d(4, 5, 7, 1)));
    BOOST_CHECK(!el.onSurface(Eigen::Vector4d(4, 5, 6, 1)));

}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(testSphere)

BOOST_AUTO_TEST_CASE(parseSphere) {
    std::string spStr = "obj sphere ID 9 mat ID 8 3 1 2 5";
    Sphere s = Sphere::fromString(spStr);
    BOOST_CHECK_EQUAL(s.id, 9);
    BOOST_CHECK_EQUAL(s.matId, 8);
    BOOST_CHECK(s.center.isApprox(Eigen::Vector4d(3, 1, 2, 1)));
    BOOST_CHECK(s.onSurface(Eigen::Vector4d(8, 1, 2, 1)));
    BOOST_CHECK(s.onSurface(Eigen::Vector4d(3, 6, 2, 1)));
    BOOST_CHECK(s.onSurface(Eigen::Vector4d(3, 1, -3, 1)));
    BOOST_CHECK(!s.onSurface(Eigen::Vector4d(3, 1, 2, 1)));
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(testCylinder)

BOOST_AUTO_TEST_CASE(parseCylinder) {
    const std::string str = "obj cylinder ID 12 mat ID 22 9 4 5 0 0 1 3 10";
    Cylinder c = Cylinder::fromString(str);
    BOOST_CHECK_EQUAL(c.id, 12);
    BOOST_CHECK_EQUAL(c.matId, 22);
    BOOST_CHECK(c.center.isApprox(Eigen::Vector4d(9, 4, 5, 1)));
    BOOST_CHECK(c.onSurface(Eigen::Vector4d(9, 4, 10, 1)));
    BOOST_CHECK(c.onSurface(Eigen::Vector4d(12, 4, 10, 1)));
    BOOST_CHECK(!c.onSurface(Eigen::Vector4d(9, 4, 9, 1)));
    const std::string negZStr = "obj cylinder ID 12 mat ID 22 9 4 5 0 0 -1 3 10";
    Cylinder negZc = Cylinder::fromString(negZStr);
    BOOST_CHECK(negZc.center.isApprox(Eigen::Vector4d(9, 4, 5, 1)));
    BOOST_CHECK(negZc.onSurface(Eigen::Vector4d(9, 4, 10, 1)));
    BOOST_CHECK(negZc.onSurface(Eigen::Vector4d(12, 4, 10, 1)));
    BOOST_CHECK(!negZc.onSurface(Eigen::Vector4d(9, 4, 9, 1)));
    const std::string xStr = "obj cylinder ID 12 mat ID 22 2 3 5 1 0 0 3 10";
    Cylinder xc = Cylinder::fromString(xStr);
    BOOST_CHECK(xc.onSurface(Eigen::Vector4d(7, 6, 5, 1)));
    BOOST_CHECK(xc.onSurface(Eigen::Vector4d(-3, 3, 5, 1)));
    BOOST_CHECK(xc.onSurface(Eigen::Vector4d(-3, 6, 5, 1)));
    BOOST_CHECK(!xc.onSurface(Eigen::Vector4d(6, 6, 6, 1)));

    std::string model1Str = "obj cylinder ID 2 mat ID 2 0 -2.5 1 0.70710678118655 0.40824829046386 0.57735026918963 0.3 2";
    Cylinder model1 = Cylinder::fromString(model1Str);

}

BOOST_AUTO_TEST_CASE(cylinderNormal) {
    std::string str = "obj cylinder ID 0 mat ID 1 0 0 0  0 0 1 1 4";
    Cylinder c = Cylinder::fromString(str);
    BOOST_CHECK(c.onSurface(Eigen::Vector4d(1, 0, -2, 1)));
    BOOST_CHECK(c.getUnitNormal(Eigen::Vector4d(1, 0, 1, 1)).isApprox(Eigen::Vector4d(1, 0, 0, 0)));

    BOOST_CHECK(c.getUnitNormal(Eigen::Vector4d(1, 0, -1, 1)).isApprox(Eigen::Vector4d(1, 0, 0, 0)));
}

BOOST_AUTO_TEST_SUITE_END()
