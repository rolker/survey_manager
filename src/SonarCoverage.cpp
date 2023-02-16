#include "survey_manager/SonarCoverage.h"
#include <pluginlib/class_list_macros.h>
#include "project11/utils.h"
#include "geographic_msgs/GeoPath.h"

PLUGINLIB_EXPORT_CLASS(survey_manager::SonarCoverage, nodelet::Nodelet)


namespace survey_manager
{
  namespace p11 = project11;

    SonarCoverage::SonarCoverage():m_interval(5.0),m_alongship_beamwidth(5.0),m_port_angle(75.0),m_starboard_angle(75.0),m_interval_accumulated_distance(0.0)
    {
        m_half_alongship_beamwidth_tan = tan(m_alongship_beamwidth*M_PI/360.0);
        m_port_tan = tan(m_port_angle*M_PI/180.0);
        m_starboard_tan = tan(m_starboard_angle*M_PI/180.0);
    }

    
    void SonarCoverage::onInit()
    {
        NODELET_DEBUG("Initializing nodelet...");
        
        auto node = getNodeHandle();
        m_depth_sub = node.subscribe("/depth",10,&SonarCoverage::depthCallback, this);
        m_heading_sub = node.subscribe("/heading",10,&SonarCoverage::headingCallback, this);
        m_position_sub = node.subscribe("/position",10,&SonarCoverage::positionCallback, this);
        m_reset_sub = node.subscribe("/sim_reset",10,&SonarCoverage::resetCallback, this);
        
        m_coverage_pub = node.advertise<geographic_msgs::GeoPath>("/coverage",10);
        //m_mbes_ping_pub = node.advertise<geographic_msgs::GeoPath>("/mbes_ping",10);
    }
    
    void SonarCoverage::depthCallback(std_msgs::Float32::ConstPtr data)
    {
        ros::Time now = ros::Time::now();
        if(now-m_last_heading_time < ros::Duration(.5) && now-m_last_position_time < ros::Duration(.5))
        {
            //std::cerr << "SonarCoverage: depth: " << data->data << ", position: " << m_latitude << ", " << m_longitude << ", heading: " << m_heading << std::endl;
            PingRecord pr;
            pr.heading = m_heading;
            pr.nadir_latitude = m_latitude;
            pr.nadir_longitude = m_longitude;
            pr.port_distance = data->data*m_port_tan;
            pr.starboard_distance = data->data*m_starboard_tan;
            if(!m_interval_record.empty())
            {
              p11::LatLongDegrees p1, p2;
              p1[0] = m_interval_record.back().nadir_latitude;
              p1[1] = m_interval_record.back().nadir_longitude;
              p2[0] = pr.nadir_latitude;
              p2[1] = pr.nadir_longitude;
              
              auto azimuth_distance = gz4d::geo::WGS84::Ellipsoid::inverse(p1,p2);
              m_interval_accumulated_distance += azimuth_distance.second;
              //std::cerr << "distance: " << m_interval_accumulated_distance << std::endl;
            }
            m_interval_record.push_back(pr);
            if(m_interval_accumulated_distance > m_interval)
                processInterval();
            
            // send ping as rectangle representing it's footprint.
            Polygon coordinates;
            p11::LatLongDegrees nadir;
            nadir[0] = pr.nadir_latitude;
            nadir[1] = pr.nadir_longitude;
            auto starboard_point = p11::WGS84::direct(nadir,p11::AngleDegrees(pr.heading+90),pr.starboard_distance);
            double alongship_half_distance = data->data * m_half_alongship_beamwidth_tan;
            //std::cerr << "alongship_half_distance: " << alongship_half_distance << std::endl;
            auto starboard_fwd_point = p11::WGS84::direct(starboard_point,p11::AngleDegrees(pr.heading),alongship_half_distance);

            boost::geometry::append(coordinates,Point(starboard_fwd_point[1],starboard_fwd_point[0]));
            auto port_fwd_point = p11::WGS84::direct(starboard_fwd_point, p11::AngleDegrees(pr.heading-90),pr.starboard_distance+pr.port_distance);
            boost::geometry::append(coordinates,Point(port_fwd_point[1],port_fwd_point[0]));
            
            auto port_back_point = p11::WGS84::direct(port_fwd_point,p11::AngleDegrees(pr.heading+180),2*alongship_half_distance);
            
            boost::geometry::append(coordinates,Point(port_back_point[1],port_back_point[0]));
             
            auto starboard_back_point = p11::WGS84::direct(port_back_point,p11::AngleDegrees(pr.heading+90),pr.starboard_distance+pr.port_distance);
            
            boost::geometry::append(coordinates,Point(starboard_back_point[1], starboard_back_point[0]));

            boost::geometry::correct(coordinates);
            
            MultiPolygon merged;
            boost::geometry::union_(coordinates,m_coverage,merged);

            m_coverage = merged;
            
            publishCoverage();
        }
        //else
        //    std::cerr << "SonarCoverage: depth: " << data->data << std::endl;
    }
    
    void SonarCoverage::headingCallback(project11_msgs::NavEulerStamped::ConstPtr data)
    {
        m_heading = data->orientation.heading;
        m_last_heading_time = data->header.stamp;
        //std::cerr << "SonarCoverage: heading: " << data->orientation.heading << std::endl;
    }

    void SonarCoverage::positionCallback(geographic_msgs::GeoPointStamped::ConstPtr data)
    {
        m_latitude = data->position.latitude;
        m_longitude = data->position.longitude;
        m_last_position_time = data->header.stamp;
        //std::cerr << "SonarCoverage: position: " << data->position.latitude << ", " << data->position.longitude << std::endl;
    }
    
    void SonarCoverage::processInterval()
    {
        if(!m_interval_record.empty())
        {
            double min_distance_port = m_interval_record.front().port_distance;
            double min_distance_starboard = m_interval_record.front().starboard_distance;
            for(auto ir: m_interval_record)
            {
                min_distance_port = std::min(min_distance_port,ir.port_distance);
                min_distance_starboard = std::min(min_distance_starboard,ir.starboard_distance);
            }
            if(m_pings.empty())
            {
                PingRecord pr = m_interval_record.front();
                pr.port_distance = min_distance_port;
                pr.starboard_distance = min_distance_starboard;
                m_pings.push_back(pr);
            }
            PingRecord pr = m_interval_record.back();
            pr.port_distance = min_distance_port;
            pr.starboard_distance = min_distance_starboard;
            m_pings.push_back(pr);
            m_interval_record.clear();
            m_interval_accumulated_distance = 0.0;
            publishCoverage();
        }
    }

    void SonarCoverage::publishCoverage()
    {
        geographic_msgs::GeoPath gpath;
        for(auto p: m_coverage)
        {
            for(auto point: p.outer())
            {
                geographic_msgs::GeoPoseStamped gpose;
                gpose.pose.position.latitude = point.y();
                gpose.pose.position.longitude = point.x();
                gpath.poses.push_back(gpose); 
            }
            geographic_msgs::GeoPoseStamped gpose;
            gpose.pose.position.latitude = -91;
            gpose.pose.position.longitude = -181;
            gpath.poses.push_back(gpose); 
        }
        m_coverage_pub.publish(gpath);
        return;

        if(!m_pings.empty())
        {
            geographic_msgs::GeoPath gpath;
            std::vector<p11::LatLongDegrees> port_positions;
            for(PingRecord p: m_pings)
            {
                p11::LatLongDegrees nadir;
                nadir[0] = p.nadir_latitude;
                nadir[1] = p.nadir_longitude;
                auto starboard_point = p11::WGS84::direct(nadir, p11::AngleDegrees(p.heading+90), p.starboard_distance);
                geographic_msgs::GeoPoseStamped gpose;
                gpose.pose.position.latitude = starboard_point[0];
                gpose.pose.position.longitude = starboard_point[1];
                gpath.poses.push_back(gpose);
                port_positions.push_back(p11::WGS84::direct(nadir,p11::AngleDegrees(p.heading-90), p.port_distance));
            }
            for(auto p = port_positions.rbegin(); p!=port_positions.rend();p++)
            {
                geographic_msgs::GeoPoseStamped gpose;
                gpose.pose.position.latitude = (*p)[0];
                gpose.pose.position.longitude = (*p)[1];
                gpath.poses.push_back(gpose);
            }
            m_coverage_pub.publish(gpath);
        }
    }
    
    void SonarCoverage::resetCallback(std_msgs::Bool::ConstPtr data)
    {
        m_interval_record.clear();
        m_interval_accumulated_distance = 0.0;
        m_pings.clear();
        publishCoverage();
    }


}
