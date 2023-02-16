#ifndef SONARCOVERAGE_H
#define SONARCOVERAGE_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <project11_msgs/NavEulerStamped.h>
#include <geographic_msgs/GeoPointStamped.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>


namespace survey_manager
{
    typedef boost::geometry::model::d2::point_xy<double> Point;
    typedef boost::geometry::model::polygon<Point> Polygon;
    typedef boost::geometry::model::multi_polygon<Polygon> MultiPolygon;
    
    class SonarCoverage: public nodelet::Nodelet
    {
    public:
        SonarCoverage();
        
        virtual void onInit();
        
        struct PingRecord
        {
            double nadir_latitude;
            double nadir_longitude;
            double heading;
            double port_distance;
            double starboard_distance;
        };
        
    private:
        void depthCallback(std_msgs::Float32::ConstPtr data);
        void headingCallback(project11_msgs::NavEulerStamped::ConstPtr data);
        void positionCallback(geographic_msgs::GeoPointStamped::ConstPtr data);
        void resetCallback(std_msgs::Bool::ConstPtr data);
        
        void processInterval();
        void publishCoverage();
        
        ros::Subscriber m_depth_sub;
        ros::Subscriber m_heading_sub;
        ros::Subscriber m_position_sub;
        ros::Subscriber m_reset_sub;
        
        ros::Publisher m_coverage_pub;
        //ros::Publisher m_mbes_ping_pub;
        
        std::vector<PingRecord> m_pings;
        
        MultiPolygon m_coverage;
        
        /// Interval in meters between pings used in coverage polygon. 
        double m_interval;

        double m_alongship_beamwidth;
        double m_port_angle;
        double m_starboard_angle;
        
        double m_half_alongship_beamwidth_tan;
        double m_port_tan;
        double m_starboard_tan;
        
        /// Raw ping records to be used at next interval point to determin minimum swath widths in latest interval.
        std::vector<PingRecord> m_interval_record;
        double m_interval_accumulated_distance;
        
        double m_longitude;
        double m_latitude;
        ros::Time m_last_position_time;
        
        double m_heading;
        ros::Time m_last_heading_time;
    };
}

#endif
