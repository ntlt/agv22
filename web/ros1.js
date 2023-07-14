// var ros
var ros,viewer,gird,tfClient,gridClient,laser,urdfClient,axes,createJoystick,urdfClient1,gridClient1
var cmd_vel_listener,move,listener,listener2,listener3
// var common
var green,yellow,red,mode,map_x=100,map_y=100
var pose,pose_pub,pose_msg
var point,point_pub,point_msg,data_co,stop_mis,stop_new
function init() {
    // creat view 
    viewer = new ROS3D.Viewer({
      divID : 'map',
      width : 1000,
      height : 1000, 
      background: '#66CDAA',
      antialias : true,
      alpha: 0.5,
      cameraPose: {x:0, y:10, z:40},
    });
    //
    // add map file 
    // gridClient = new ROS3D.OccupancyGridClient({
    //     ros : ros,
    //     rootObject : viewer.scene,
    //     //continuous: false,
    //     tfClient: tfClient,
    //     //topic: '/MB21_916b/map',
    //     topic: '/map',
    // });
    gridClient = new ROS3D.OccupancyGridClient({
      ros : ros,
      rootObject : viewer.scene,
      continuous: true,
      tfClient: tfClient,
      topic: '/MB21_916b/map',
      //topic: '/map',
      continuous: true,
    });
    // 
    
    // // add grid 
    // grid = new ROS3D.Grid({
    //     num_cells : 1000,
    //     color : '#888888',
    //     lineWidth : 1,
    //     cellSize : 1
    // });
    // viewer.addObject(grid);
    //

    // axes origin
    axes=new ROS3D.Axes({
        scale: 1,
        shaftRadius: 0.025,
        headRadius: 0.05,
    });
    // axes.x=10;
    // axes.y=10;
    viewer.addObject(axes);
    //

    // add map frame
    tfClient = new ROSLIB.TFClient({
        ros : ros,
        rate : 10,
        fixedFrame : '/map',
        //queue_size: 1,
        angularThres : 0.08,
        transThres : 0.05,
        });
  //   //
  //   tfClient.subscribe('/MB21_916b/base_footprint', function(tf) {
  //     //console.log(tf);
  //     //laser.subscribe();
  //     //alert("a");
  //   });
  //   path = new ROS3D.Path({
  //     ros : ros,
  //     rootObject : viewer.scene,
  //     tfClient : tfClient,
  //     topic: "/MB21_916b/move_base/NavfnROS/plan",
  //     // queue_size: 1,
  //     // throttle_rate: 1000,	
  //     // color: 0x0000FF,
  //   });
  //   // add laser view
    laser = new ROS3D.LaserScan({
        ros : ros,
        topic: "/MB21_916b/laser/scan",
        rootObject : viewer.scene,
        tfClient: tfClient,   
        material : { size: 0.5, color: 0xff0000 },
        //queue_size: 1,
        rate : 1,
    });
  //   //
  //   // add model robot
    urdfClient = new ROS3D.UrdfClient({
        ros : ros,
        tfClient : tfClient,
        param: '/MB21_916b/robot_description',
        //path : 'http://10.0.1.2:8000/mvibot2.urdf',
        rootObject : viewer.scene,
        loader : ROS3D.COLLADA_LOADER_2,
    });
    urdfClient1 = new ROS3D.UrdfClient({
      ros : ros,
      tfClient : tfClient,
      param: '/test/robot_description',
      // path : '192.168.1.0:9090',
      rootObject : viewer.scene,
      loader : ROS3D.COLLADA_LOADER_2,
   });

  //   // topic cmd vel for robot
    cmd_vel_listener = new ROSLIB.Topic({
        ros : ros,
        name : "/cmd_vel",
        messageType : 'geometry_msgs/Twist',
        queue_size: 1,
    });
  //   //

  //   // topic map size
  //   map = new ROSLIB.Topic({
  //       ros : ros,
  //       name : "/MB21_916b/map",
  //       messageType : 'nav_msgs/OccupancyGrid',
  //   });
  //   map.subscribe(function(message) {
  //   //   map_x=message.info.width/2*message.info.resolution;
  //   //   map_y=message.info.height/2*message.info.resolution;
  //        gridClient.unsubscribe();
  //   });
    //
    data_co=new ROSLIB.Topic({
      ros : ros,
      name : "/MB21_916b/data_coordinates",
      messageType : 'std_msgs/String',
    });
    stop_mis=new ROSLIB.Topic({
      ros : ros,
      name : "/MB21_916b/mission_cancel",
      messageType : 'std_msgs/String',
    });
    stop_new=new ROSLIB.Topic({
      ros : ros,
      name : "/MB21_916b/mission_continue",
      //name : "/MB21_916b/mission_cancel",
      messageType : 'std_msgs/String',
    });
    // listen mode operating robot
    // listener_mode = new ROSLIB.Topic({
    //     ros : ros,
    //     name : '/mode',
    //     messageType : 'std_msgs/String',
    //     queue_size: 1,
	  //   throttle_rate: 1000,
    // });
    // listener_mode.subscribe(function(message) {
    //     mode=message.data;
    // });
  // listener_scan = new ROSLIB.Topic({
  //     ros : ros,
  //     name : '/MB21_916b/scan',
  //     messageType : 'sensor_msgs/LaserScan',
  // });
  // listener_scan.subscribe(function(message) {
  //     //mode=message.data;
  //     //alert("A");
  //     //laser.unsubscribe();
  // });
    //
    move = function (linear, angular) {
      var twist = new ROSLIB.Message({
          linear: {
            x: linear,
            y: 0,
            z: 0
          },
          angular: {
            x: 0,
            y: 0,
            z: angular
          }
        });
        cmd_vel_listener.publish(twist);
    }
    //
    createJoystick = function () {
          var options = {
            zone: document.getElementById('zone_joystick'),
            threshold: 0.1,
            position: {
                   left: 1450 + 'px',
                   top: 350 + 'px' },
            mode: 'static',
            size: 400,
            color: 'black',
          };
          manager = nipplejs.create(options);
          linear_speed = 0;
          angular_speed = 0;
          self.manager.on('start', function (event, nipple) {
            console.log("Movement start");
              timer = setInterval(function () {
            move(linear_speed, angular_speed);
          }, 500);
          });
          self.manager.on('move', function (event, nipple) {
              console.log("Moving");
              max_linear = 0.4; // m/s
          max_angular = 0.314; // rad/s
          max_distance = 200; // pixels;
          linear_speed = Math.sin(nipple.angle.radian) * max_linear * nipple.distance/max_distance;
          angular_speed = -Math.cos(nipple.angle.radian) * max_angular * nipple.distance/max_distance;
          });
          self.manager.on('end', function () {
            console.log("Movement end");
          if (timer) {
          clearInterval(timer);
          } 
          self.move(0, 0);
          });
    }
    createJoystick();
    //
 }

 // function connecting to robot
function connecting() {
  ros = new ROSLIB.Ros({
  url: 'ws://'+ip+':9090'
  });
  //alert(ros.url);
  ros.on('connection', function() {
    alert("Connected");
  });

  ros.on('close', function() {
    alert("Disconnected");
    location.reload();
  });
}
