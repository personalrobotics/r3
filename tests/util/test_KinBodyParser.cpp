#include <gtest/gtest.h>
#include <aikido/util/KinBodyParser.hpp>

#define STR_EXPAND(tok) #tok
#define STR(tok) STR_EXPAND(tok)

using namespace aikido::util;

static std::string TEST_RESOURCES_PATH = STR(AIKIDO_TEST_RESOURCES_PATH);

//==============================================================================
TEST(KinBodyParser, LoadBoxGeomFromString)
{
  const std::string str =
      "<KinBody name=\"paper_box\">                \n"
      "  <Body type=\"static\" name=\"paper_box\"> \n"
      "    <Geom type=\"box\">                     \n"
      "      <Translation>0 0 0</Translation>      \n"
      "      <Extents>0.1 0.145 0.055</Extents>    \n"
      "      <DiffuseColor>0 0 1.0</DiffuseColor>  \n"
      "    </Geom>                                 \n"
      "  </Body>                                   \n"
      "</KinBody>                                    ";

  auto skel = readKinbodyString(str);
  EXPECT_TRUE(skel != nullptr);
  EXPECT_TRUE(skel->getNumBodyNodes() == 1u);
  EXPECT_TRUE(skel->getNumJoints() == 1u);

  auto bodyNode = skel->getBodyNode(0);
  EXPECT_TRUE(bodyNode != nullptr);

  auto shapeNodes = bodyNode->getShapeNodes();
  EXPECT_TRUE(shapeNodes.size() == 1u);

  auto shapeNode = shapeNodes[0];
  EXPECT_TRUE(shapeNode != nullptr);

  auto shape = shapeNode->getShape();
  EXPECT_TRUE(shape->is<dart::dynamics::BoxShape>());

  auto boxShape = static_cast<dart::dynamics::BoxShape*>(shape.get());
  EXPECT_TRUE(boxShape->getSize().isApprox(Eigen::Vector3d(0.1, 0.145, 0.055)));

  auto joint = skel->getJoint(0);
  EXPECT_TRUE(joint != nullptr);
}

//==============================================================================
TEST(KinBodyParser, LoadBoxGeom)
{
  auto uri = std::string("file://")
      + TEST_RESOURCES_PATH
      + std::string("/kinbody/objects/block.kinbody.xml");
  EXPECT_TRUE(dart::common::LocalResourceRetriever().exists(uri));

  auto skel = readKinbody(uri);
  EXPECT_TRUE(skel != nullptr);
  EXPECT_TRUE(skel->getNumBodyNodes() == 1u);
  EXPECT_TRUE(skel->getNumJoints() == 1u);

  auto bodyNode = skel->getBodyNode(0);
  EXPECT_TRUE(bodyNode != nullptr);

  auto shapeNodes = bodyNode->getShapeNodes();
  EXPECT_TRUE(shapeNodes.size() == 1u);

  auto shapeNode = shapeNodes[0];
  EXPECT_TRUE(shapeNode != nullptr);

  auto shape = shapeNode->getShape();
  EXPECT_TRUE(shape->is<dart::dynamics::BoxShape>());

  auto boxShape = static_cast<dart::dynamics::BoxShape*>(shape.get());
  EXPECT_TRUE(boxShape->getSize().isApprox(
      Eigen::Vector3d(0.0127, 0.0127, 0.0127)));

  auto joint = skel->getJoint(0);
  EXPECT_TRUE(joint != nullptr);
}

//==============================================================================
TEST(KinBodyParser, LoadSphereGeom)
{
  auto uri = std::string("file://")
      + TEST_RESOURCES_PATH
      + std::string("/kinbody/objects/smallsphere.kinbody.xml");
  EXPECT_TRUE(dart::common::LocalResourceRetriever().exists(uri));

  auto skel = readKinbody(uri);
  EXPECT_TRUE(skel != nullptr);
  EXPECT_TRUE(skel->getNumBodyNodes() == 1u);
  EXPECT_TRUE(skel->getNumJoints() == 1u);

  auto bodyNode = skel->getBodyNode(0);
  EXPECT_TRUE(bodyNode != nullptr);

  auto shapeNodes = bodyNode->getShapeNodes();
  EXPECT_TRUE(shapeNodes.size() == 1u);

  auto shapeNode = shapeNodes[0];
  EXPECT_TRUE(shapeNode != nullptr);

  auto shape = shapeNode->getShape();
  EXPECT_TRUE(shape->is<dart::dynamics::SphereShape>());

  auto sphereShape = static_cast<dart::dynamics::SphereShape*>(shape.get());
  EXPECT_NEAR(sphereShape->getRadius(), 0.013, 1e-12);

  auto joint = skel->getJoint(0);
  EXPECT_TRUE(joint != nullptr);
}

//==============================================================================
TEST(KinBodyParser, LoadCylinderGeom)
{
  auto uri = std::string("file://")
      + TEST_RESOURCES_PATH
      + std::string("/kinbody/objects/stamp.kinbody.xml");
  EXPECT_TRUE(dart::common::LocalResourceRetriever().exists(uri));

  auto skel = readKinbody(uri);
  EXPECT_TRUE(skel != nullptr);
  EXPECT_TRUE(skel->getNumBodyNodes() == 1u);
  EXPECT_TRUE(skel->getNumJoints() == 1u);

  auto bodyNode = skel->getBodyNode(0);
  EXPECT_TRUE(bodyNode != nullptr);

  auto shapeNodes = bodyNode->getShapeNodes();
  EXPECT_TRUE(shapeNodes.size() == 1u);

  auto shapeNode = shapeNodes[0];
  EXPECT_TRUE(shapeNode != nullptr);

  auto shape = shapeNode->getShape();
  EXPECT_TRUE(shape->is<dart::dynamics::CylinderShape>());

  auto cylinderShape = static_cast<dart::dynamics::CylinderShape*>(shape.get());
  EXPECT_NEAR(cylinderShape->getRadius(), 0.02, 1e-12);
  EXPECT_NEAR(cylinderShape->getHeight(), 0.20, 1e-12);

  auto joint = skel->getJoint(0);
  EXPECT_TRUE(joint != nullptr);
}

//==============================================================================
TEST(KinBodyParser, LoadTriMeshGeomBowl)
{
  auto uri = std::string("file://")
      + TEST_RESOURCES_PATH
      + std::string("/kinbody/objects/bowl.kinbody.xml");
  EXPECT_TRUE(dart::common::LocalResourceRetriever().exists(uri));

  auto skel = readKinbody(uri);
  EXPECT_TRUE(skel != nullptr);
  EXPECT_TRUE(skel->getNumBodyNodes() == 1u);
  EXPECT_TRUE(skel->getNumJoints() == 1u);

  auto bodyNode = skel->getBodyNode(0);
  EXPECT_TRUE(bodyNode != nullptr);

  auto shapeNodes = bodyNode->getShapeNodes();
  EXPECT_TRUE(shapeNodes.size() == 1u);

  auto shapeNode = shapeNodes[0];
  EXPECT_TRUE(shapeNode != nullptr);

  auto shape = shapeNode->getShape();
  EXPECT_TRUE(shape->is<dart::dynamics::MeshShape>());

  auto meshShape = static_cast<dart::dynamics::MeshShape*>(shape.get());
  EXPECT_TRUE(!meshShape->getMeshUri().empty());
  EXPECT_TRUE(meshShape->getScale().isApprox(Eigen::Vector3d::Constant(1.0)));

  auto joint = skel->getJoint(0);
  EXPECT_TRUE(joint != nullptr);
}

//==============================================================================
TEST(KinBodyParser, LoadTriMeshGeomKinovaTool)
{
  auto uri = std::string("file://")
      + TEST_RESOURCES_PATH
      + std::string("/kinbody/objects/kinova_tool.kinbody.xml");
  EXPECT_TRUE(dart::common::LocalResourceRetriever().exists(uri));

  auto skel = readKinbody(uri);
  EXPECT_TRUE(skel != nullptr);
  EXPECT_TRUE(skel->getNumBodyNodes() == 1u);
  EXPECT_TRUE(skel->getNumJoints() == 1u);

  auto bodyNode = skel->getBodyNode(0);
  EXPECT_TRUE(bodyNode != nullptr);

  auto shapeNodes = bodyNode->getShapeNodes();
  EXPECT_EQ(shapeNodes.size(), 3u);

  auto shapeNode1 = shapeNodes[0];
  auto shapeNode2 = shapeNodes[1];
  auto shapeNode3 = shapeNodes[2];
  EXPECT_TRUE(shapeNode1 != nullptr);
  EXPECT_TRUE(shapeNode2 != nullptr);
  EXPECT_TRUE(shapeNode3 != nullptr);

  auto shape1 = shapeNode1->getShape();
  auto shape2 = shapeNode2->getShape();
  auto shape3 = shapeNode3->getShape();
  EXPECT_TRUE(shape1->is<dart::dynamics::SphereShape>());
  EXPECT_TRUE(shape2->is<dart::dynamics::MeshShape>());
  EXPECT_TRUE(shape3->is<dart::dynamics::MeshShape>());

  auto sphereShape1 = static_cast<dart::dynamics::SphereShape*>(shape1.get());
  auto meshShape2 = static_cast<dart::dynamics::MeshShape*>(shape2.get());
  auto meshShape3 = static_cast<dart::dynamics::MeshShape*>(shape3.get());
  EXPECT_TRUE(sphereShape1->getRadius() == 0.0);
  EXPECT_TRUE(!meshShape2->getMeshUri().empty());
  EXPECT_TRUE(!meshShape3->getMeshUri().empty());
  EXPECT_TRUE(meshShape2->getScale().isApprox(Eigen::Vector3d::Constant(1.0)));
  EXPECT_TRUE(meshShape3->getScale().isApprox(Eigen::Vector3d::Constant(0.1)));

  auto joint = skel->getJoint(0);
  EXPECT_TRUE(joint != nullptr);
}
