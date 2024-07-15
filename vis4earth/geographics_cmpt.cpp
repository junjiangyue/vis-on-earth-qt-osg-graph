﻿#include <vis4earth/geographics_cmpt.h>

#include <ui_geographics_cmpt.h>

VIS4Earth::GeographicsComponent::GeographicsComponent(QWidget *parent)
    : QtOSGReflectableWidget(ui, parent) {
    for (auto name : {"longtitudeMin", "longtitudeMax", "latitudeMin", "latitudeMax"}) {
        // 为经纬度属性设置 角度转弧度 的转换器
        auto &prop = properties.at(name);
        prop->SetConvertor([](Reflectable::Type val) -> Reflectable::Type {
            assert(val.type == Reflectable::ESupportedType::Float);
            return Reflectable::Type(Math::DegToRad(val.val.asFloat));
        });
    }
    for (auto name : {"heightMin", "heightMax"}) {
        // 将地表高度转换为地心高度
        auto &prop = properties.at(name);
        prop->SetConvertor([](Reflectable::Type val) -> Reflectable::Type {
            assert(val.type == Reflectable::ESupportedType::Float);
            return Reflectable::Type(val.val.asFloat +
                                     static_cast<float>(osg::WGS_84_RADIUS_POLAR));
        });
    }

    rotMat = new osg::Uniform("rotMat", osg::Matrix3());
    auto onGeographicsChanged = [&](double) {
        auto lonMin = GetPropertyOSGValue<float>("longtitudeMin");
        auto lonMax = GetPropertyOSGValue<float>("longtitudeMax");
        auto latMin = GetPropertyOSGValue<float>("latitudeMin");
        auto latMax = GetPropertyOSGValue<float>("latitudeMax");

        auto lon = .5f * (lonMin.val + lonMax.val);
        auto lat = .5f * (latMin.val + latMax.val);
        auto h = 1.f;

        osg::Vec3 dir;
        dir.z() = h * sin(lat);
        h = h * cos(lat);
        dir.y() = h * sin(lon);
        dir.x() = h * cos(lon);
        dir *= -1.f;
        dir.normalize();

        osg::Matrix3 rotMat;
        rotMat(2, 0) = dir.x();
        rotMat(2, 1) = dir.y();
        rotMat(2, 2) = dir.z();
        auto tmp = osg::Vec3(0.f, 0.f, 1.f);
        tmp = tmp ^ dir;
        rotMat(0, 0) = tmp.x();
        rotMat(0, 1) = tmp.y();
        rotMat(0, 2) = tmp.z();
        tmp = dir ^ tmp;
        rotMat(1, 0) = tmp.x();
        rotMat(1, 1) = tmp.y();
        rotMat(1, 2) = tmp.z();

        this->rotMat->set(rotMat);

        emit GeographicsChanged();
    };
    connect(ui->doubleSpinBox_longtitudeMin_float_VIS4EarthReflectable,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged), onGeographicsChanged);
    connect(ui->doubleSpinBox_longtitudeMax_float_VIS4EarthReflectable,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged), onGeographicsChanged);
    connect(ui->doubleSpinBox_latitudeMin_float_VIS4EarthReflectable,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged), onGeographicsChanged);
    connect(ui->doubleSpinBox_latitudeMax_float_VIS4EarthReflectable,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged), onGeographicsChanged);
    connect(ui->doubleSpinBox_heightMin_float_VIS4EarthReflectable,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged), onGeographicsChanged);
    connect(ui->doubleSpinBox_heightMax_float_VIS4EarthReflectable,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged), onGeographicsChanged);
    onGeographicsChanged(0.);
}
