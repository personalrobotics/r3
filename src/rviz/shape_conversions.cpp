#include "aikido/rviz/shape_conversions.hpp"

#include <Eigen/StdVector>
#include <assimp/cexport.h>
#include <boost/filesystem.hpp>
#include <dart/dynamics/dynamics.hpp>

#include "aikido/rviz/ResourceServer.hpp"

using dart::dynamics::BoxShape;
using dart::dynamics::CylinderShape;
using dart::dynamics::EllipsoidShape;
using dart::dynamics::LineSegmentShape;
using dart::dynamics::MeshShape;
using dart::dynamics::PlaneShape;
using dart::dynamics::Shape;
using dart::dynamics::SoftMeshShape;
using dart::dynamics::SphereShape;
using visualization_msgs::Marker;

namespace aikido {
namespace rviz {

//==============================================================================
geometry_msgs::Point convertEigenToROSPoint(const Eigen::Vector3d& v)
{
  geometry_msgs::Point v_ros;
  v_ros.x = v[0];
  v_ros.y = v[1];
  v_ros.z = v[2];
  return v_ros;
}

//==============================================================================
geometry_msgs::Vector3 convertEigenToROSVector3(const Eigen::Vector3d& v)
{
  geometry_msgs::Vector3 v_ros;
  v_ros.x = v[0];
  v_ros.y = v[1];
  v_ros.z = v[2];
  return v_ros;
}

//==============================================================================
std_msgs::ColorRGBA convertEigenToROSColorRGBA(const Eigen::Vector4d& v)
{
  std_msgs::ColorRGBA v_ros;
  v_ros.r = v[0];
  v_ros.g = v[1];
  v_ros.b = v[2];
  v_ros.a = v[3];
  return v_ros;
}

//==============================================================================
Eigen::Vector4d convertROSColorRGBAToEigen(const std_msgs::ColorRGBA& v)
{
  Eigen::Vector4d vec4;
  vec4[0] = v.r;
  vec4[1] = v.g;
  vec4[2] = v.b;
  vec4[3] = v.a;
  return vec4;
}

//==============================================================================
geometry_msgs::Quaternion convertEigenToROSQuaternion(
    const Eigen::Quaterniond& v)
{
  geometry_msgs::Quaternion v_ros;
  v_ros.w = v.w();
  v_ros.x = v.x();
  v_ros.y = v.y();
  v_ros.z = v.z();
  return v_ros;
}

//==============================================================================
geometry_msgs::Pose convertEigenToROSPose(const Eigen::Isometry3d& v)
{
  Eigen::Quaterniond const quaternion(v.rotation());

  geometry_msgs::Pose v_ros;
  v_ros.orientation = convertEigenToROSQuaternion(quaternion);
  v_ros.position = convertEigenToROSPoint(v.translation());
  return v_ros;
}

//==============================================================================
bool convertAssimpMeshToROSTriangleList(
    const aiMesh& mesh, std::vector<geometry_msgs::Point>* triangle_list)
{
  triangle_list->reserve(triangle_list->size() + 3 * mesh.mNumFaces);

  for (unsigned int iface = 0; iface < mesh.mNumFaces; ++iface)
  {
    const aiFace& face = mesh.mFaces[iface];

    if (face.mNumIndices != 3)
    {
      return false;
    }

    for (unsigned int i = 0; i < 3; ++i)
    {
      unsigned int ivertex = face.mIndices[i];
      const aiVector3D& vertex = mesh.mVertices[ivertex];

      geometry_msgs::Point ros_vertex;
      ros_vertex.x = vertex.x;
      ros_vertex.y = vertex.y;
      ros_vertex.z = vertex.z;

      triangle_list->push_back(ros_vertex);
    }
  }
  return true;
}

//==============================================================================
bool convertShape(
    const BoxShape& shape, Marker* marker, ResourceServer* /*resourceManager*/)
{
  marker->type = Marker::CUBE;
  marker->pose.orientation.w = 1.;
  marker->scale = convertEigenToROSVector3(shape.getSize());
  return true;
}

//==============================================================================
bool convertShape(
    const CylinderShape& shape,
    Marker* marker,
    ResourceServer* /*resourceManager*/)
{
  marker->type = Marker::CYLINDER;
  marker->pose.orientation.w = 1.;
  marker->scale.x = 2. * shape.getRadius();
  marker->scale.y = 2. * shape.getRadius();
  marker->scale.z = shape.getHeight();
  return true;
}

//==============================================================================
bool convertShape(
    const EllipsoidShape& shape,
    Marker* marker,
    ResourceServer* /*resourceManager*/)
{
  marker->type = Marker::SPHERE;
  marker->pose.orientation.w = 1.;
  marker->scale = convertEigenToROSVector3(shape.getDiameters());
  return true;
}

//==============================================================================
bool convertShape(
    const LineSegmentShape& shape,
    Marker* marker,
    ResourceServer* /*resourceManager*/)
{
  const std::vector<Eigen::Vector3d>& vertices = shape.getVertices();
  const std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>>&
      connections
      = shape.getConnections();

  marker->type = Marker::LINE_STRIP;
  marker->pose.orientation.w = 1.;
  marker->scale.x = shape.getThickness();
  marker->points.reserve(2 * connections.size());

  for (const Eigen::Vector2i& connection : connections)
  {
    const std::size_t& i1 = connection[0];
    const std::size_t& i2 = connection[1];

    if (i1 >= connections.size() || i2 >= connections.size())
    {
      return false;
    }

    const Eigen::Vector3d& p1 = vertices[i1];
    const Eigen::Vector3d& p2 = vertices[i2];
    marker->points.push_back(convertEigenToROSPoint(p1));
    marker->points.push_back(convertEigenToROSPoint(p2));
  }
  return true;
}

//==============================================================================
bool convertShape(
    const MeshShape& shape, Marker* marker, ResourceServer* /*resourceManager*/)
{
  marker->pose.orientation.w = 1.;
  marker->scale = convertEigenToROSVector3(shape.getScale());

  const aiScene* scene = shape.getMesh();
  const std::string& meshPath = shape.getMeshPath();
  if (!meshPath.empty())
  {
    marker->type = Marker::MESH_RESOURCE;
    marker->mesh_resource = meshPath;
    marker->mesh_use_embedded_materials = true;
    return true;
  }

  // Fall back on publishing the mesh as a TRIANGLE_LIST.
  if (scene)
  {
    marker->type = Marker::TRIANGLE_LIST;

    for (unsigned int imesh = 0; imesh < scene->mNumMeshes; ++imesh)
    {
      if (!convertAssimpMeshToROSTriangleList(
              *scene->mMeshes[imesh], &marker->points))
      {
        return false;
      }
    }
    return true;
  }

  return false; // Everything failed!
}

//==============================================================================
bool convertShape(
    const PlaneShape& shape,
    Marker* marker,
    ResourceServer* /*resourceManager*/,
    double width)
{
  static Eigen::Matrix<double, 3, 6> const points
      = (Eigen::Matrix<double, 6, 3>() << -0.5,
         -0.5,
         0,
         0.5,
         0.5,
         0,
         -0.5,
         0.5,
         0,
         -0.5,
         -0.5,
         0,
         0.5,
         -0.5,
         0,
         0.5,
         0.5,
         0)
            .finished()
            .transpose();

  marker->type = Marker::TRIANGLE_LIST;

  const Eigen::Vector3d& normal = shape.getNormal();
  const double offset = shape.getOffset();

  // Constructing a frame on the plane at the point closest to the origin.
  Eigen::Quaterniond quat;
  quat.setFromTwoVectors(Eigen::Vector3d::UnitZ(), normal);

  Eigen::Affine3d frame(quat);
  frame.pretranslate(normal * offset);
  frame.scale(width);

  Eigen::MatrixXd const transformedPoints = frame * points;
  marker->points.resize(transformedPoints.cols());

  for (auto i = 0u; i < static_cast<std::size_t>(transformedPoints.cols()); ++i)
    marker->points[i] = convertEigenToROSPoint(transformedPoints.col(i));

  return true;
}

//==============================================================================
bool convertShape(
    const SoftMeshShape& shape,
    Marker* marker,
    ResourceServer* /*resourceManager*/)
{
  marker->type = Marker::TRIANGLE_LIST;
  marker->pose.orientation.w = 1.;

  aiMesh const* mesh = shape.getAssimpMesh();
  if (mesh)
  {
    return convertAssimpMeshToROSTriangleList(*mesh, &marker->points);
  }
  else
  {
    return false;
  }
}

//==============================================================================
bool convertShape(
    const SphereShape& shape,
    Marker* marker,
    ResourceServer* /*resourceManager*/)
{
  marker->type = Marker::SPHERE;
  marker->pose.orientation.w = 1.;
  marker->scale.x = 2. * shape.getRadius();
  marker->scale.y = 2. * shape.getRadius();
  marker->scale.z = 2. * shape.getRadius();
  return true;
}

//==============================================================================
bool convertShape(const Shape& shape, Marker* marker, ResourceServer* rm)
{
  if (shape.is<BoxShape>())
    return convertShape(dynamic_cast<const BoxShape&>(shape), marker, rm);

  else if (shape.is<EllipsoidShape>())
    return convertShape(dynamic_cast<const EllipsoidShape&>(shape), marker, rm);

  else if (shape.is<CylinderShape>())
    return convertShape(dynamic_cast<const CylinderShape&>(shape), marker, rm);

  else if (shape.is<LineSegmentShape>())
    return convertShape(
        dynamic_cast<const LineSegmentShape&>(shape), marker, rm);

  else if (shape.is<MeshShape>())
    return convertShape(dynamic_cast<const MeshShape&>(shape), marker, rm);

  else if (shape.is<PlaneShape>())
    return convertShape(dynamic_cast<const PlaneShape&>(shape), marker, rm);

  else if (shape.is<SoftMeshShape>())
    return convertShape(dynamic_cast<const SoftMeshShape&>(shape), marker, rm);

  else if (shape.is<SphereShape>())
    return convertShape(dynamic_cast<const SphereShape&>(shape), marker, rm);

  else
    return false;
}

} // namespace rviz
} // namespace aikido
