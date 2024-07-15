﻿#ifndef VIS4EARTH_SCALAR_VISER_ISOSURFACE_H
#define VIS4EARTH_SCALAR_VISER_ISOSURFACE_H

#include <set>
#include <vector>

#include <osg/CoordinateSystemNode>
#include <osg/CullFace>
#include <osg/Group>
#include <osg/ShapeDrawable>

#include <vis4earth/geographics_cmpt.h>
#include <vis4earth/osg_util.h>
#include <vis4earth/qt_osg_reflectable.h>
#include <vis4earth/volume_cmpt.h>

#include <vis4earth/scalar_viser/marching_cube_table.h>

namespace Ui {
class IsosurfaceRenderer;
}

namespace VIS4Earth {

class IsosurfaceRenderer : public QtOSGReflectableWidget {
    Q_OBJECT

  public:
    enum class EMeshSmoothType { None, Laplacian, Curvature };

    IsosurfaceRenderer(QWidget *parent = nullptr);

    osg::ref_ptr<osg::Group> GetGroup() const { return grp; }

  private:
    uint8_t isoval;
    bool useVolSmoothed;
    EMeshSmoothType meshSmoothType;

    Ui::IsosurfaceRenderer *ui;
    GeographicsComponent geoCmpt;
    VolumeComponent volCmpt;

    osg::ref_ptr<osg::Group> grp;
    osg::ref_ptr<osg::Geometry> geom;
    osg::ref_ptr<osg::Geode> geode;
    osg::ref_ptr<osg::Program> program;
    osg::ref_ptr<osg::Vec3Array> verts;
    osg::ref_ptr<osg::Vec3Array> vertSmootheds;
    osg::ref_ptr<osg::Vec3Array> norms;
    osg::ref_ptr<osg::Vec3Array> normSmootheds;
    osg::ref_ptr<osg::Vec2Array> uvs;

    osg::ref_ptr<osg::Uniform> eyePos;

    std::vector<GLuint> vertIndices;
    std::array<std::set<std::array<GLuint, 2>>, 2> multiEdges;

    void initOSGResource();

    void marchingCube(uint32_t volID);

    void updateGeometry(uint32_t volID);
};

} // namespace VIS4Earth

#endif // !VIS4EARTH_SCALAR_VISER_ISOSURFACE_H
