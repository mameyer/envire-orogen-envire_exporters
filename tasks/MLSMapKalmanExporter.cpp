/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MLSMapKalmanExporter.hpp"
#include <boost/filesystem.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>

using namespace envire_exporters;

MLSMapKalmanExporter::MLSMapKalmanExporter(std::string const& name, TaskCore::TaskState initial_state)
    : MLSMapKalmanExporterBase(name, initial_state)
{
}

MLSMapKalmanExporter::MLSMapKalmanExporter(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : MLSMapKalmanExporterBase(name, engine, initial_state)
{
}

MLSMapKalmanExporter::~MLSMapKalmanExporter()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MLSMapKalmanExporter.hpp for more detailed
// documentation about them.

bool MLSMapKalmanExporter::configureHook()
{
    std::string path = _path.get();
    
    if(!boost::filesystem::exists(path))
    {
        std::cout << "Error, path " << path << " does not exist " << std::endl;
        return false;
    }

    if(path.find(".ply") != std::string::npos)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PLYReader plyReader;
        if(plyReader.read(path, *cloud) >= 0)
        {
            pcl::PointXYZ mi, ma; 
            pcl::getMinMax3D (*cloud, mi, ma); 
            
            double mls_res = _mls_resolution.get();
            double size_x = std::max(ma.x, -std::min<float>(mi.x, 0.0)) * 2.0;
            double size_y = std::max(ma.y, -std::min<float>(mi.y, 0.0)) * 2.0;
            
            std::cout << "MIN: " << mi << ", MAX: " << ma << std::endl;
            maps::grid::MLSConfig cfg;
            map = maps::grid::MLSMapKalman(maps::grid::Vector2ui(size_x / mls_res, size_y / mls_res), maps::grid::Vector2d(mls_res, mls_res), cfg);
            map.translate(base::Vector3d(- size_x / 2.0, - size_y / 2.0, 0));
            base::TransformWithCovariance tf = base::TransformWithCovariance::Identity();
            tf.cov.setZero();
            map.mergePointCloud(*cloud, tf);
        }
    }
    else if (path.find(".bin") != std::string::npos)
    {
        std::ifstream mlsStream(path);
        if (mlsStream.is_open())
        {
            boost::archive::binary_iarchive mlsIn(mlsStream);
            mlsIn >> map;
        }
    }
    else
    {
        std::cout << "Error, only ply loading supported atm" << std::endl;
        return false;
    }
    
    
    if (! MLSMapKalmanExporterBase::configureHook())
        return false;

    return true;
}
bool MLSMapKalmanExporter::startHook()
{
    written = false;
    if (! MLSMapKalmanExporterBase::startHook())
        return false;
    return true;
}
void MLSMapKalmanExporter::updateHook()
{
    
    
    
    if(!written)
    {
        writeMap();
        written = true;
    }
    MLSMapKalmanExporterBase::updateHook();
}

void MLSMapKalmanExporter::writeMap()
{
    envire::core::SpatioTemporal<maps::grid::MLSMapKalman> mapFoo(map);
    mapFoo.setFrameID("Map");
    mapFoo.setTime(base::Time::now());

    _map.write(mapFoo);    
}
