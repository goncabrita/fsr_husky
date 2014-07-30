#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/cache.h>
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/signals.hpp>
#include <boost/bind.hpp>
#include <boost/mem_fn.hpp>
#include <geometry_msgs/PointStamped.h>
#include <metal_detector_msgs/Coil.h>
#include <rti/rti.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;
using namespace sensor_msgs;
using namespace message_filters;
using namespace metal_detector_msgs;

//Parametros para krigging
#define MY_RTI_CELL_RADIUS 0.2 //0.2raio da celula (igual ou menor ao raio do circulo?)
#define MY_RTI_NUMBER_OF_VARIOGRAMS 1
#define MY_RTI_RADIUS 0.4      //0.3raio de cada circulo de interpolacao (em metros)
#define MY_RTI_RESOLUTION 0.05 //diminuir este numero fica mais refinado (em metros)
#define MY_RTI_UPDATE_FREQUENCY 5 //5Hz
#define X_OFFSET 0 //para evitar indices negativos na matrix do RTI
#define Y_OFFSET 0 
#define MY_RAW_ESCALA 1000.0
#define MY_RTI_CELL_SIZE 1

/*

//Parametros para IDW
#define MY_RTI_CELL_RADIUS 0.2 //0.2raio da celula (igual ou menor ao raio do circulo?)
#define MY_RTI_NUMBER_OF_VARIOGRAMS 1
#define MY_RTI_RADIUS 0.4      //0.3raio de cada circulo de interpolacao (em metros)
#define MY_RTI_RESOLUTION 0.03 //diminuir este numero fica mais refinado (em metros)
#define MY_RTI_UPDATE_FREQUENCY 5 //5Hz
#define X_OFFSET 0 //para evitar indices negativos na matrix do RTI
#define Y_OFFSET 0 
#define MY_RAW_ESCALA 10000.0
#define MY_RTI_CELL_SIZE 10
*/

// --- p Kriging
 map_data  mapa_local(0, 0, MY_RTI_CELL_SIZE, MY_RTI_CELL_SIZE, MY_RTI_RESOLUTION, MY_RTI_RESOLUTION);      // Starts at 0,0, cells with 5 x 5cm and resolution of 0.1m
// Input: 	x_init_center = first cell center x
//			y_init_center = first cell center y
//			x_cell = cell width
//			y_cell = cell height
//			res_x = resolution in x (meters per index)
//			res_y = resolution in y (meters per index)
map_data	interpolacao(0, 0, MY_RTI_CELL_SIZE, MY_RTI_CELL_SIZE, MY_RTI_RESOLUTION, MY_RTI_RESOLUTION);
map_data    error_interp(0, 0, MY_RTI_CELL_SIZE, MY_RTI_CELL_SIZE, MY_RTI_RESOLUTION, MY_RTI_RESOLUTION);

// --- For rviz
ros::Publisher marker_pub;
visualization_msgs::Marker mapa_plot, marker, points, line_strip, line_list;
		
	
class MetalDetector
{
public:
    MetalDetector() : tf_(),  target_frame_("minefield")
    {
        md_sub_.subscribe(n_, "coils", 10);
        tf_filter_ = new tf::MessageFilter<metal_detector_msgs::Coil>(md_sub_, tf_, target_frame_, 10);
        tf_filter_->registerCallback( boost::bind(&MetalDetector::msgCallback, this, _1) );
        pub_ = n_.advertise<geometry_msgs::PointStamped>("coil_position", 10);
        
        //inicializacao das variaveis
     	qtd_pontos = 0;
		last_X = 0;
		last_Y = 0;
		last_Z = 0;
		inicio = 1;

    }
    
    private:
    message_filters::Subscriber<metal_detector_msgs::Coil> md_sub_;
    tf::TransformListener tf_;
    tf::MessageFilter<metal_detector_msgs::Coil> * tf_filter_;
    ros::NodeHandle n_;
    std::string target_frame_;
    ros::Publisher pub_;
    ros::Publisher * pontos_joao_ptr;//APAGAR DEPOIS
	// --- p Kriging
	//point3d robot;
	//list<point3d> buffer;
	long int qtd_pontos;
	char filename[30];
	//----
	float last_X, last_Y, last_Z;
	int inicio;

	// - Rainbow Color Transformation ---
	typedef struct {
		double r;       // percent
		double g;       // percent
		double b;       // percent
	} rgb;

	typedef struct {
		double h;       // angle in degrees
		double s;       // percent
		double v;       // percent
	} hsv;

//METHODS
rgb hsv2rgb(hsv in)
{
    double      hh, p, q, t, ff;
    long        i;
    rgb         out;

    if(in.s <= 0.0) {       // < is bogus, just shuts up warnings
        if(isnan(in.h)) {   // in.h == NAN
            out.r = in.v;
            out.g = in.v;
            out.b = in.v;
            return out;
        }
        // error - should never happen
        out.r = 0.0;
        out.g = 0.0;
        out.b = 0.0;
        return out;
    }
    hh = in.h;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = in.v * (1.0 - in.s);
    q = in.v * (1.0 - (in.s * ff));
    t = in.v * (1.0 - (in.s * (1.0 - ff)));

    switch(i) {
    case 0:
        out.r = in.v;
        out.g = t;
        out.b = p;
        break;
    case 1:
        out.r = q;
        out.g = in.v;
        out.b = p;
        break;
    case 2:
        out.r = p;
        out.g = in.v;
        out.b = t;
        break;

    case 3:
        out.r = p;
        out.g = q;
        out.b = in.v;
        break;
    case 4:
        out.r = t;
        out.g = p;
        out.b = in.v;
        break;
    case 5:
    default:
        out.r = in.v;
        out.g = p;
        out.b = q;
        break;
    }
    return out;
}


//This function receives a value in any range defined by max and min values, and create a color object when min is blue and max is red
std_msgs::ColorRGBA RangeValue_to_RainbowColor(int * my_hue, long int maxvalue, long int minvalue, long int CoilM_C0){

    long int valor = CoilM_C0 - minvalue;
    long int cor = valor * 240 / (maxvalue-minvalue); //para ficar entre 0 e 240, é a parte que quero do (hsv color space que vai de 0-360)

    hsv hsv_color;
    hsv_color.h = cor; //para ir de vermelho para azul - roxo
     //240-cor; //para ir de azul a vermelho
    hsv_color.s = 0.9;
    hsv_color.v = 0.9;

    //outro retorno
    *my_hue = hsv_color.h;

    rgb rgb_color;
    rgb_color = hsv2rgb(hsv_color);


    std_msgs::ColorRGBA c2;
    c2.r = rgb_color.r;
    c2.g = rgb_color.g;
    c2.b = rgb_color.b;
    c2.a = 1.0;

    return c2;
}


	
void publishRvizMarker(float X,float Y, float Z, std_msgs::ColorRGBA corPonto){
	
	
	geometry_msgs::Point p;
	geometry_msgs::Point p2;

	
	
   //---------RVIZ Marker publication---------------
    //turn Global --- visualization_msgs::Marker marker, line_strip, line_list;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/minefield";
    line_strip.header.frame_id = "/minefield";
    line_list.header.frame_id = "/minefield";
    points.header.frame_id = "/minefield";

    marker.header.stamp = points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = points.ns = line_strip.ns = line_list.ns = "Red_to_3D";
    marker.action = points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;

// %Tag(NS_ID)%
    marker.id = 0;
    line_strip.id=1;
    line_list.id=2;
    points.id = 3;
// %EndTag(NS_ID)%

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
// %Tag(TYPE)%
    marker.type = visualization_msgs::Marker::SPHERE;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    points.type = visualization_msgs::Marker::POINTS;
// %EndTag(TYPE)%

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.05;
    points.scale.y = 0.05; //0.1

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
// %Tag(POSE)%
    marker.pose.position.x = X;
    marker.pose.position.y = Y;
    marker.pose.position.z = Z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
// %EndTag(POSE)%

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
// %Tag(SCALE)%
    marker.scale.x = 0.3; //0.5
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    //--LINE_STRIP and LINE_LIST markers use only the x component of the scale, for the line width
    line_strip.scale.x = 0.01;
    line_list.scale.x = 0.01;
// %EndTag(SCALE)%

    // Set the color -- be sure to set alpha to something non-zero!
// %Tag(COLOR)%
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    //--- line strip is blue


    // Points are initially red
    points.color.r = 1.0;
    points.color.a = 1.0;

    // --- Line Strip is Light Gray
    line_strip.color.r = 0.2;
    line_strip.color.g = 0.2;
    line_strip.color.b = 0.2;
    line_strip.color.a = 0.5;
    //--- line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    
// %EndTag(COLOR)%

// %Tag(LIFETIME)%
    marker.lifetime = ros::Duration();


// %EndTag(LIFETIME)%
	
	
	p.x = X;
	p.y = Y;
	p.z = Z;

	p2.x = X;
	p2.y = Y;
	p2.z = 0;


	line_strip.points.push_back(p);

	points.points.push_back(p2);
	points.colors.push_back(corPonto);


	// Publish the marker
    fprintf(stderr,"\npublish marker\n");
    //marker_pub.publish(line_strip);
    //marker_pub.publish(line_list);
    marker_pub.publish(points);

}


/*float lastLX = 0;
float lastLY = 0;
float L_inicio = 1;
float lastRX = 0;
float lastRY = 0;
float R_inicio = 1;
float stop_left = 0;
float stop_right = 0;
*/

 void msgCallback(const boost::shared_ptr<const metal_detector_msgs::Coil>& coil_ptr)
    {
        geometry_msgs::PointStamped point_in;
        point_in.header.frame_id = coil_ptr->header.frame_id;
        point_in.header.stamp = coil_ptr->header.stamp;
        point_in.point.x = 0.0;
        point_in.point.y = 0.0;
        point_in.point.z = 0.0;

        geometry_msgs::PointStamped point_out;


        try
        {
            tf_.transformPoint(target_frame_, point_in, point_out);

            // Note that z is the position of the coil, not the position of the possible metal sample!
             ROS_INFO("Coil %s with data ch0 %d ch1 %d ch2 %d at x %f y %f z %f",
                coil_ptr->header.frame_id.c_str(),
                coil_ptr->channel[0],
                coil_ptr->channel[1],
                coil_ptr->channel[2],
                point_out.point.x,
                point_out.point.y,
                point_out.point.z);

             point_out.header.frame_id = coil_ptr->header.frame_id;
             point_out.header.stamp = coil_ptr->header.stamp;

	//point_out tem a posicao
	//coil_ptr->channel[0]
            
	long int minvalue = 0; 
    long int maxvalue = 1;

	float mapX; //para evitar indices negativos na matriz
	float mapY;

	std_msgs::ColorRGBA corPonto;
	
	//fprintf(stderr,"\nX=%lf,Y=%lf,Z=%lf\n",MD_position->point.x,MD_position->point.y,MD_position->point.z);

			fprintf(stderr,"%d",coil_ptr->channel[0]);

			    // --- preparando o arco-iris ---
                     
                        //if(coil_ptr->zero[0]==0){
                               //long int minvalue = -1930000; //fixando zero como 2 milhoes (para este dataset), assim tenho apenas valores positivos (o certo é pegar o menor valor de todo o scan para ser o zero)
                            minvalue = -300000; //fixando zero como 2 milhoes (para este dataset), assim tenho apenas valores positivos (o certo é pegar o menor valor de todo o scan para ser o zero)
                            maxvalue =  -150000;
                        //}else{
                        //    minvalue = coil_ptr->zero[0];
                        //    maxvalue = -150000;
                        //}
                        
                        int my_hue=0; //vai ser modificado pela função a seguir para conter o valor da coil, normalizado entre 0 e 240


                        corPonto = RangeValue_to_RainbowColor(&my_hue,maxvalue,minvalue,coil_ptr->channel[0]);
                        
                      
                        //-------------------------------

			//Interpolacao dos dados

		     mapX = point_out.point.x + X_OFFSET; //para evitar indices negativos na matriz
			 mapY = point_out.point.y + Y_OFFSET;

		    //guardar pontos -- krigering
            //MD
            if(qtd_pontos%6==0){ //reduzir os pontos fornecidos por 3 para o rti ir mais longe sem encravar
				mapa_local.set_robot_position(mapX, mapY);
				mapa_local.insert_robot_sample(mapX, mapY, (coil_ptr->channel[0]/MY_RAW_ESCALA) );
			}
            qtd_pontos++;
			//end -- krigering

			
			publishRvizMarker(point_out.point.x,   point_out.point.y,   (coil_ptr->channel[0]/MY_RAW_ESCALA),   corPonto);
   

            // pub_.publish( point_out ); 

        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("Failure %s\n", ex.what());
        }
    }


}; //fim da classe metal detector

 int main(int argc, char **argv)
 {
   fprintf(stderr,"\nStarted...\n");

   ros::init(argc, argv, "Point_and_MD_listener");
   ros::NodeHandle nh;
   ros::Rate r(10);
   
   
   // --- Kriging
   interpolacao.set_variogram_cell_radius(MY_RTI_CELL_RADIUS); 
   interpolacao.set_number_of_variograms(MY_RTI_NUMBER_OF_VARIOGRAMS);  

   ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud> ("points2", 10);
   ros::Publisher pcl_pub2 = nh.advertise<sensor_msgs::PointCloud> ("points2_error", 10);
   
   //APAGAR DEPOIS
   /*ros::Publisher pontos_joao = nh.advertise<geometry_msgs::Point> ("geo_point", 1000);
   pontos_joao_ptr = &pontos_joao;*/

   boost::thread_group threads;

   //SET FRAME
   interpolacao.set_frame_name("/minefield");
   error_interp.set_frame_name("/minefield");
   mapa_local.set_frame_name("/minefield"); 
   //RADIUS
   mapa_local.set_radius(MY_RTI_RADIUS); //15
   mapa_local.set_inter_pcl_pub(pcl_pub);
   mapa_local.set_estimation_variance_pcl_pub(pcl_pub2);

   
   //mapa_local.start_IDW_interpolation(threads,interpolacao,MY_RTI_UPDATE_FREQUENCY,0);
 
   mapa_local.start_Kriging_interpolation(threads,interpolacao,error_interp,MY_RTI_UPDATE_FREQUENCY,0);  //10, era 2
   
   // --- end krigring ---------


   //Publisher of marker for rviz
   marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
   //PointCloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> ("points2", 1);

 
   //COM MESSAGE FILTERS 
 /*  message_filters::Subscriber<geometry_msgs::PointStamped> position_sub(nh, "/coil_position", 1);
   message_filters::Subscriber<Coil> coils_sub(nh, "/coils", 1); //middle coil only
   
   //for MD and odor sensor
   typedef sync_policies::ApproximateTime<geometry_msgs::PointStamped,Coil> MySyncPolicy;
   Synchronizer<MySyncPolicy> sync(MySyncPolicy(30), position_sub, coils_sub);
   sync.registerCallback(boost::bind( &myCallback , _1, _2));
   */
  
   //COM MESSAGE FILTERS e TF listener
    
	 MetalDetector md;
        
        
   ros::spin();
   
   threads.join_all();

  
 }
