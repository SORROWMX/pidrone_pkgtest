/**
* Setup all visualization elements when the page is loaded.
*/

function empty(element) {
    while (element.firstChild) {
      element.removeChild(element.firstChild);
    }
}

function printProperties(obj) {
    for(var propt in obj){
      console.log(propt + ': ' + obj[propt]);
    }
}

function myround(number, precision) {
  var factor = Math.pow(10, precision);
  var tempNumber = number * factor;
  var roundedTempNumber = Math.round(tempNumber);
  return roundedTempNumber / factor;
};

var markerClient;
var ros;
var modepub;
var modeMsg;
var positionMsg;
var twistMsg;
var poseMsg;
var positionPub;
var positionControlPub;
var velocityControlPub;
var heartbeatPub;
var heightChart;
var velocityChart;
var windowSize = 5;
var gotFirstHeight = false;
var gotFirstVelocity = false;
var startTime;
var heightChartPaused = false;
var velocityChartPaused = false;
var showingUkfAnalysis = false;
var spanningFullWindow = false;

// Добавляем переменные для отслеживания последнего режима и дебаунсинга
var lastPositionMode = null;
var positionModeDebounceTimer = null;
var notificationDebounceTime = 500; // ms

// Добавляем переменные для ArUco маркеров
var markersInfoSub;
var lastMarkersInfo = null;
var markersUpdateTimer = null;

function closeSession(){
  console.log("Closing connections.");
  if (ros) {
    ros.close();
    // Обновляем состояние кнопок
    toggleConnectionButtons(false);
  }
  return false;
}

/* This code runs when you click 'connect' */
function connect() {
    // Connect to ROS.
    if(ros && ros.isConnected) {
        return;
    }
    
    var url = 'ws://' + document.getElementById('hostname').value + ':9090';
    ros = new ROSLIB.Ros({
        url : url
    });

    var velocityBtn = document.getElementById('velocityBtn');
    velocityBtn.addEventListener("click", publishVelocityMode, false);

    var positionBtn = document.getElementById('positionBtn');
    positionBtn.addEventListener("click", publishPositionMode, false);

    ros.on('error', function(error) {
      console.log('ROS Master:  Error, check console.');
      //printProperties(error);
      document.getElementById('statusMessage').innerHTML='<span class="status-indicator status-disconnected mr-2"></span>Ошибка подключения';
      $('#statusMessage').addClass('alert-danger').removeClass('alert-success');
      // Обновляем состояние кнопок
      toggleConnectionButtons(false);
    });

    ros.on('connection', function() {
      console.log('ROS Master:  Connected.');
      //printProperties(error);
      document.getElementById('statusMessage').innerHTML='<span class="status-indicator status-connected mr-2"></span>Подключено';
      $('#statusMessage').addClass('alert-success').removeClass('alert-danger');
      // Обновляем состояние кнопок
      toggleConnectionButtons(true);
      
      // Инициализация новых функций
      setupStateSubscription();
      setupMarkersSubscription(); // Добавляем подписку на информацию о маркерах
    });

    ros.on('close', function() {
      console.log('ROS Master:  Connection closed.');
      //printProperties(error);
      document.getElementById('statusMessage').innerHTML='<span class="status-indicator status-disconnected mr-2"></span>Отключено';
      $('#statusMessage').addClass('alert-danger').removeClass('alert-success');
      // Обновляем состояние кнопок
      toggleConnectionButtons(false);
    });

    /*
     * ROS Messages
     */

    modeMsg = new ROSLIB.Message({
      mode: "DISARMED",
     });

    emptyMsg = new ROSLIB.Message({
    });

    positionMsg = new ROSLIB.Message({
        // default is velocity mode
        data : false
    });

    poseMsg = new ROSLIB.Message({
        position : {
            x : 0.0,
            y : 0.0,
            z : 0.0
        },
        orientation : {
            x : 0.0,
            y : 0.0,
            z : 0.0,
            w : 0.0
        }
    });

    twistMsg = new ROSLIB.Message({
        linear : {
            x : 0.0,
            y : 0.0,
            z : 0.0
        },
        angular : {
            x : 0.0,
            y : 0.0,
            z : 0.0
        }
    });

    /*
     * ROS Publishers
     */

    modepub = new ROSLIB.Topic({
      ros : ros,
      name : '/pidrone/desired/mode',
      messageType : 'pidrone_pkg/Mode'
    });

    heartbeatPub = new ROSLIB.Topic({
      ros : ros,
      name : '/pidrone/heartbeat/web_interface',
      messageType : 'std_msgs/Empty'
    });

    setInterval(function(){
      heartbeatPub.publish(emptyMsg);
      //console.log("heartbeat");
    }, 1000);

    positionPub = new ROSLIB.Topic({
        ros : ros,
        name : '/pidrone/position_control',
        messageType : 'std_msgs/Bool'
    });

    velocityControlPub = new ROSLIB.Topic({
        ros : ros,
        name : '/pidrone/desired/twist',
        messageType : 'geometry_msgs/Twist'
    });

    positionControlPub = new ROSLIB.Topic({
        ros : ros,
        name : '/pidrone/desired/pose',
        messageType : 'geometry_msgs/Pose'
    });

    resetpub = new ROSLIB.Topic({
      ros : ros,
      name : '/pidrone/reset_transform',
      messageType : 'std_msgs/Empty'
    });

    mappub = new ROSLIB.Topic({
        ros : ros,
        name : '/pidrone/map',
        messageType : 'std_msgs/Empty'
    })

    togglepub = new ROSLIB.Topic({
      ros : ros,
      name : '/pidrone/toggle_transform',
      messageType : 'std_msgs/Empty'
    });

    /*
     * ROS Subscribers
     */

    positionSub = new ROSLIB.Topic({
        ros : ros,
        name : '/pidrone/position_control',
        messageType : 'std_msgs/Bool',
        queue_length : 2,
        throttle_rate : 80
    });

    // TODO: Merge with code that has Battery.msg
    // (published from flight controller node)
    batterysub = new ROSLIB.Topic({
      ros : ros,
      name : '/pidrone/battery',
      messageType : 'pidrone_pkg/Battery',
      queue_length : 2,
      throttle_rate : 2
    });

    irsub = new ROSLIB.Topic({
      ros : ros,
      name : '/pidrone/range',
      messageType : 'sensor_msgs/Range',
      queue_length : 2,
      throttle_rate : 80
    });


    velsub = new ROSLIB.Topic({
      ros : ros,
      name : '/pidrone/picamera/twist',
      messageType : 'geometry_msgs/TwistStamped',
      queue_length : 2,
      throttle_rate : 80
    });

    ukf2dsub = new ROSLIB.Topic({
        ros : ros,
        name : '/pidrone/state/ukf_2d',
        messageType : 'pidrone_pkg/State',
        queue_length : 2,
        throttle_rate : 80
    });
    
    ukf7dsub = new ROSLIB.Topic({
        ros : ros,
        name : '/pidrone/state/ukf_7d',
        messageType : 'pidrone_pkg/State',
        queue_length : 2,
        throttle_rate : 80
    });

    cameraPoseSub = new ROSLIB.Topic({
        ros : ros,
        name : '/pidrone/picamera/pose',
        messageType : 'geometry_msgs/PoseStamped',
        queue_length : 2,
        throttle_rate : 80
    });

    emaIrSub = new ROSLIB.Topic({
      ros : ros,
      name : '/pidrone/state/ema',
      messageType : 'pidrone_pkg/State',
      queue_length : 2,
      throttle_rate : 80
    });

    stateGroundTruthSub = new ROSLIB.Topic({
        ros : ros,
        name : '/pidrone/state/ground_truth',
        messageType : 'pidrone_pkg/StateGroundTruth',
        queue_length : 2,
        throttle_rate : 80
    });
    
    ukfStatsSub = new ROSLIB.Topic({
        ros : ros,
        name : '/pidrone/ukf_stats',
        messageType : 'pidrone_pkg/UkfStats',
        queue_length : 2,
        throttle_rate : 80
    });

    // Barometer altitude data
    barosub = new ROSLIB.Topic({
        ros : ros,
        name : '/pidrone/altitude',
        messageType : 'std_msgs/Float32',
        queue_length : 2,
        throttle_rate : 80
    });

    // Подписываемся на информацию о маркерах
    markersInfoSub = new ROSLIB.Topic({
        ros : ros,
        name : '/drone_vision/markers_info',
        messageType : 'std_msgs/String',
        queue_length : 1
    });

    /*
     * ROS Subscriber Callbacks
     */

     positionSub.subscribe(function(message) {
        var position = message.data;
        
        // Проверяем, изменился ли режим с момента последнего обновления
        if (lastPositionMode === position) {
            return; // Пропускаем обновление, если режим не изменился
        }
        
        // Сбрасываем предыдущий таймер дебаунса, если он существует
        if (positionModeDebounceTimer) {
            clearTimeout(positionModeDebounceTimer);
        }
        
        // Устанавливаем таймер дебаунса
        positionModeDebounceTimer = setTimeout(function() {
            // Обновляем отслеживание последнего режима
            lastPositionMode = position;
            
            var text = "";
            if (position) {
                text = '<div class="alert alert-position"><span class="mode-indicator mode-position"></span><strong>РЕЖИМ ПОЗИЦИОНИРОВАНИЯ</strong> <i class="fas fa-map-marker-alt ml-2"></i></div>';
                // Обновляем уведомление для XY Chart при переключении в режим позиционирования
                $('#xy_chart_info').removeClass('alert-warning').addClass('alert-info')
                    .html('<i class="fas fa-info-circle"></i> Данные о позиции активны. Карта обновляется.');
                
                // Обновляем кнопки
                $('#velocityBtn').removeClass('active btn-velocity').addClass('btn-secondary');
                $('#positionBtn').removeClass('btn-secondary').addClass('active btn-position');
                
                // Обновляем стили инпутов
                $('.control-input').removeClass('velocity').addClass('position');
                
                // Показываем уведомление
                showNotification('Включен режим позиционирования', 'success');
            } else {
                text = '<div class="alert alert-velocity"><span class="mode-indicator mode-velocity"></span><strong>РЕЖИМ УПРАВЛЕНИЯ СКОРОСТЬЮ</strong> <i class="fas fa-tachometer-alt ml-2"></i></div>';
                // Обновляем уведомление для XY Chart при переключении в режим скорости
                $('#xy_chart_info').removeClass('alert-info').addClass('alert-warning')
                    .html('<i class="fas fa-exclamation-triangle"></i> Внимание: Данные о позиции недоступны в режиме скорости.');
                
                // Обновляем кнопки
                $('#positionBtn').removeClass('active btn-position').addClass('btn-secondary');
                $('#velocityBtn').removeClass('btn-secondary').addClass('active btn-velocity');
                
                // Обновляем стили инпутов
                $('.control-input').removeClass('position').addClass('velocity');
                
                // Показываем уведомление
                showNotification('Включен режим управления скоростью', 'info');
            }
            element = document.getElementById("position_state");
            element.innerHTML = text;
            
            // Добавляем анимацию для привлечения внимания
            $('#position_state').fadeOut(100).fadeIn(100).fadeOut(100).fadeIn(100);
        }, notificationDebounceTime);
    });

    batterysub.subscribe(function(message) {
      //printProperties(message);
      var voltage = message.vbat;
      var mynumber = myround(voltage, 2);
      
      // Автоматическое определение типа батареи и установка порогов
      var lowVoltageThreshold;
      var batteryType = "";
      
      if (voltage > 15.0) { // Вероятно 4S (16.8V при полной зарядке)
          lowVoltageThreshold = 14.0; // ~3.5V на ячейку для 4S
          batteryType = "4S";
      } else { // Предполагаем 3S (12.6V при полной зарядке)
          lowVoltageThreshold = 11.1; // ~3.7V на ячейку для 3S
          batteryType = "3S";
      }
      
      document.getElementById('vbat').innerHTML = mynumber + "V " + batteryType;
      
      if (voltage <= lowVoltageThreshold) {
        document.getElementById('vbat').innerHTML = mynumber + "V " + batteryType + " LOW!";
        $('#vbat').addClass('alert-danger').removeClass('alert-success');
      } else {
        $('#vbat').addClass('alert-success').removeClass('alert-danger');
      }
    });

    // Обработчик сообщений с информацией о маркерах
    markersInfoSub.subscribe(function(message) {
        try {
            var markersData = JSON.parse(message.data);
            lastMarkersInfo = markersData;
            
            // Обновляем UI с информацией о маркерах
            updateMarkersUI(markersData);
        } catch (e) {
            console.error('Ошибка при обработке данных о маркерах:', e);
        }
    });

    var heightChartMinTime;
    var heightChartMaxTime;
    irsub.subscribe(function(message) {
      //printProperties(message);
      //console.log("Range: " + message.range);
      currTime = message.header.stamp.secs + message.header.stamp.nsecs/1.0e9;
      if (!gotFirstHeight) {
          gotFirstHeight = true;
          startTime = currTime;
      }
      tVal = currTime - startTime;
      // Have the plot scroll in time, showing a window of windowSize seconds
      if (tVal > windowSize) {
          spanningFullWindow = true;
          heightChartMinTime = tVal - windowSize;
          heightChartMaxTime = tVal;
          // Remove first element of array while difference compared to current
          // time is greater than the windowSize
          while (rawIrData.length > 0 &&
                 (tVal - rawIrData[0].x > windowSize)) {
              rawIrData.splice(0, 1);
          }
      }
      // Add new range reading to end of the data array
      // x-y pair
      var xyPair = {
          x: tVal,
          y: message.range
      }
      rawIrData.push(xyPair)
      if (!heightChartPaused && !showingUkfAnalysis) {
          heightChart.options.scales.xAxes[0].ticks.min = heightChartMinTime;
          heightChart.options.scales.xAxes[0].ticks.max = heightChartMaxTime;
          heightChart.data.datasets[0].data = rawIrData.slice();
          heightChart.update();
      } else if (!showingUkfAnalysis) {
          pulseIr();
      }
      //console.log("Data: " + heightChart.data.datasets[0].data);
      //console.log('tVal: ' + tVal)
    });

    var velocityChartMinTime;
    var velocityChartMaxTime;
    velsub.subscribe(function(message) {
      //printProperties(message);
	//console.log("Range: " + message.twist.linear.x);
	//console.log("Range: " + message.twist.linear.y);
	vel = Math.sqrt(message.twist.linear.x**2 + message.twist.linear.y**2)
      currTime = message.header.stamp.secs + message.header.stamp.nsecs/1.0e9;
      if (!gotFirstVelocity) {
          gotFirstVelocity = true;
          startTime = currTime;
      }
      tVal = currTime - startTime;
      // Have the plot scroll in time, showing a window of windowSize seconds
      if (tVal > windowSize) {
          spanningFullWindow = true;
          velocityChartMinTime = tVal - windowSize;
          velocityChartMaxTime = tVal;
          // Remove first element of array while difference compared to current
          // time is greater than the windowSize
          while (rawVelocityData.length > 0 &&
                 (tVal - rawVelocityData[0].x > windowSize)) {
              rawVelocityData.splice(0, 1);
          }
      }
      // Add new range reading to end of the data array
      // x-y pair
      var xyPair = {
          x: tVal,
          y: vel
      }
      rawVelocityData.push(xyPair)
      if (!velocityChartPaused && !showingUkfAnalysis) {
          velocityChart.options.scales.xAxes[0].ticks.min = velocityChartMinTime;
          velocityChart.options.scales.xAxes[0].ticks.max = velocityChartMaxTime;
          velocityChart.data.datasets[0].data = rawVelocityData.slice();
          velocityChart.update();
      }
    });

    function ukfCallback(message) {
      //printProperties(message);
      currTime = message.header.stamp.secs + message.header.stamp.nsecs/1.0e9;
      if (!gotFirstHeight) {
          gotFirstHeight = true;
          startTime = currTime;
      }
      tVal = currTime - startTime;
      // Have the plot scroll in time, showing a window of windowSize seconds
      if (tVal > windowSize) {
          spanningFullWindow = true;
          // Avoid changing axis limits too often, to avoid shaky plotting?
          // heightChartMinTime = tVal - windowSize;
          // heightChartMaxTime = tVal;

          // Remove first element of array while difference compared to current
          // time is greater than the windowSize
          while (ukfData.length > 0 &&
                 (tVal - ukfData[0].x > windowSize)) {
              ukfData.splice(0, 1);
              ukfPlusSigmaData.splice(0, 1);
              ukfMinusSigmaData.splice(0, 1);
          }
      }
      // Add new height estimate to end of the data array
      // x-y pair
      var zEstimate = message.pose_with_covariance.pose.position.z;
      var xyPair = {
          x: tVal,
          y: zEstimate
      }
      ukfData.push(xyPair);
      // Also plot +/- one standard deviation:
      var heightVariance = message.pose_with_covariance.covariance[14];
      var heightStdDev = Math.sqrt(heightVariance);
      var xyPairStdDevPlus = {
          x: tVal,
          y: zEstimate + heightStdDev
      }
      var xyPairStdDevMinus = {
          x: tVal,
          y: zEstimate - heightStdDev
      }
      ukfPlusSigmaData.push(xyPairStdDevPlus);
      ukfMinusSigmaData.push(xyPairStdDevMinus);
      updateUkfXYChart(message);
      if (!heightChartPaused && !showingUkfAnalysis) {
          // heightChart.options.scales.xAxes[0].ticks.min = heightChartMinTime;
          // heightChart.options.scales.xAxes[0].ticks.max = heightChartMaxTime;
          heightChart.data.datasets[1].data = ukfData.slice();
          heightChart.data.datasets[2].data = ukfPlusSigmaData.slice();
          heightChart.data.datasets[3].data = ukfMinusSigmaData.slice();
          // Avoid updating too often, to avoid shaky plotting?
          // heightChart.update();
      }
    };

    ukf2dsub.subscribe(ukfCallback);
    ukf7dsub.subscribe(ukfCallback);
    
    ukfStatsSub.subscribe(function(message) {
        currTime = message.header.stamp.secs + message.header.stamp.nsecs/1.0e9;
        if (!gotFirstHeight) {
            gotFirstHeight = true;
            startTime = currTime;
        }
        tVal = currTime - startTime;
        // Have the plot scroll in time, showing a window of windowSize seconds
        if (tVal > windowSize) {
            spanningFullWindow = true;
            heightChartMinTime = tVal - windowSize;
            heightChartMaxTime = tVal;
            
            // Remove first element of array while difference compared to current
            // time is greater than the windowSize
            while (residualData.length > 0 &&
                   (tVal - residualData[0].x > windowSize)) {
                residualData.splice(0, 1);
                plusSigmaData.splice(0, 1);
                minusSigmaData.splice(0, 1);
            }
        }
        // Add new height error to end of the data array
        // x-y pair
        var zError = message.error;
        var xyPair = {
            x: tVal,
            y: zError
        }
        residualData.push(xyPair);
        // Also plot +/- one standard deviation:
        var heightStdDev = message.stddev;
        var xyPairStdDevPlus = {
            x: tVal,
            y: heightStdDev
        }
        var xyPairStdDevMinus = {
            x: tVal,
            y: -heightStdDev
        }
        plusSigmaData.push(xyPairStdDevPlus);
        minusSigmaData.push(xyPairStdDevMinus);
        if (!heightChartPaused && showingUkfAnalysis) {
            heightChart.options.scales.xAxes[0].ticks.min = heightChartMinTime;
            heightChart.options.scales.xAxes[0].ticks.max = heightChartMaxTime;
            heightChart.data.datasets[0].data = residualData.slice();
            heightChart.data.datasets[1].data = plusSigmaData.slice();
            heightChart.data.datasets[2].data = minusSigmaData.slice();
            heightChart.update();
        }
    });
    
    cameraPoseSub.subscribe(function(message) {
        updateCameraPoseXYChart(message);
    });

    function updateGroundTruthXYChart(msg) {
        xPos = msg.pose.position.x;
        yPos = msg.pose.position.y;
        qx = msg.pose.orientation.x;
        qy = msg.pose.orientation.y;
        qz = msg.pose.orientation.z;
        qw = msg.pose.orientation.w;

        if (xPos != null &&
            yPos != null &&
            qx != null &&
            qy != null &&
            qz != null &&
            qw != null) {

            // Quaternion with which to rotate vectors to show the yaw of the
            // drone (and perhaps also the roll and pitch)
            global_to_body_quat = new Quaternion([qw, qx, qy, qz]);
            // v1 = [1, 1, 0];
            // v2 = [1, -1, 0];
            // Drone marker vectors
            v1 = [0.03, 0.03, 0];
            v2 = [0.03, -0.03, 0];
            v3 = [0.0, 0.05, 0];
            rotatedv1 = global_to_body_quat.rotateVector(v1);
            rotatedv2 = global_to_body_quat.rotateVector(v2);
            rotatedv3 = global_to_body_quat.rotateVector(v3);
            xyChart.data.datasets[0].data = [{
                x: (xPos - rotatedv1[0]),
                y: (yPos - rotatedv1[1])
            },
            {
                x: (xPos + rotatedv1[0]),
                y: (yPos + rotatedv1[1])
            }
            ];
            xyChart.data.datasets[1].data = [{
                x: (xPos - rotatedv2[0]),
                y: (yPos - rotatedv2[1])
            },
            {
                x: (xPos + rotatedv2[0]),
                y: (yPos + rotatedv2[1])
            }
            ];
            xyChart.data.datasets[2].data = [{
                x: xPos,
                y: yPos
            },
            {
                x: (xPos + rotatedv3[0]),
                y: (yPos + rotatedv3[1])
            }
            ];
            xyChart.update()
        }
    }

    function updateCameraPoseXYChart(msg) {
        xPos = msg.pose.position.x;
        yPos = msg.pose.position.y;
        qx = msg.pose.orientation.x;
        qy = msg.pose.orientation.y;
        qz = msg.pose.orientation.z;
        qw = msg.pose.orientation.w;

        if (xPos != null &&
            yPos != null &&
            qx != null &&
            qy != null &&
            qz != null &&
            qw != null) {

            // Quaternion with which to rotate vectors to show the yaw of the
            // drone (and perhaps also the roll and pitch)
            global_to_body_quat = new Quaternion([qw, qx, qy, qz]);
            // v1 = [1, 1, 0];
            // v2 = [1, -1, 0];
            // Drone marker vectors
            v1 = [0.03, 0.03, 0];
            v2 = [0.03, -0.03, 0];
            v3 = [0.0, 0.05, 0];
            rotatedv1 = global_to_body_quat.rotateVector(v1);
            rotatedv2 = global_to_body_quat.rotateVector(v2);
            rotatedv3 = global_to_body_quat.rotateVector(v3);
            xyChart.data.datasets[6].data = [{
                x: (xPos - rotatedv1[0]),
                y: (yPos - rotatedv1[1])
            },
            {
                x: (xPos + rotatedv1[0]),
                y: (yPos + rotatedv1[1])
            }
            ];
            xyChart.data.datasets[7].data = [{
                x: (xPos - rotatedv2[0]),
                y: (yPos - rotatedv2[1])
            },
            {
                x: (xPos + rotatedv2[0]),
                y: (yPos + rotatedv2[1])
            }
            ];
            xyChart.data.datasets[8].data = [{
                x: xPos,
                y: yPos
            },
            {
                x: (xPos + rotatedv3[0]),
                y: (yPos + rotatedv3[1])
            }
            ];
            xyChart.update()
        }
    }

    function updateUkfXYChart(msg) {
        xPos = msg.pose_with_covariance.pose.position.x;
        yPos = msg.pose_with_covariance.pose.position.y;
        qx = msg.pose_with_covariance.pose.orientation.x;
        qy = msg.pose_with_covariance.pose.orientation.y;
        qz = msg.pose_with_covariance.pose.orientation.z;
        qw = msg.pose_with_covariance.pose.orientation.w;

        if (xPos != null &&
            yPos != null &&
            qx != null &&
            qy != null &&
            qz != null &&
            qw != null) {

            // Quaternion with which to rotate vectors to show the yaw of the
            // drone (and perhaps also the roll and pitch)
            global_to_body_quat = new Quaternion([qw, qx, qy, qz]);
            // v1 = [1, 1, 0];
            // v2 = [1, -1, 0];
            // Drone marker vectors
            v1 = [0.03, 0.03, 0];
            v2 = [0.03, -0.03, 0];
            v3 = [0.0, 0.05, 0];
            rotatedv1 = global_to_body_quat.rotateVector(v1);
            rotatedv2 = global_to_body_quat.rotateVector(v2);
            rotatedv3 = global_to_body_quat.rotateVector(v3);
            xyChart.data.datasets[3].data = [{
                x: (xPos - rotatedv1[0]),
                y: (yPos - rotatedv1[1])
            },
            {
                x: (xPos + rotatedv1[0]),
                y: (yPos + rotatedv1[1])
            }
            ];
            xyChart.data.datasets[4].data = [{
                x: (xPos - rotatedv2[0]),
                y: (yPos - rotatedv2[1])
            },
            {
                x: (xPos + rotatedv2[0]),
                y: (yPos + rotatedv2[1])
            }
            ];
            xyChart.data.datasets[5].data = [{
                x: xPos,
                y: yPos
            },
            {
                x: (xPos + rotatedv3[0]),
                y: (yPos + rotatedv3[1])
            }
            ];
            xyChart.update()
        }
    }

    emaIrSub.subscribe(function(message) {
      //printProperties(message);
      //console.log("Range: " + message.range);
      currTime = message.header.stamp.secs + message.header.stamp.nsecs/1.0e9;
      if (!gotFirstHeight) {
          gotFirstHeight = true;
          startTime = currTime;
      }
      tVal = currTime - startTime;
      // Have the plot scroll in time, showing a window of windowSize seconds
      if (tVal > windowSize) {
          spanningFullWindow = true;
          // Avoid changing axis limits too often, to avoid shaky plotting?
          // heightChartMinTime = tVal - windowSize;
          // heightChartMaxTime = tVal;

          // Remove first element of array while difference compared to current
          // time is greater than the windowSize
          while (emaData.length > 0 &&
                 (tVal - emaData[0].x > windowSize)) {
              emaData.splice(0, 1);
          }
      }
      // Add new range reading to end of the data array
      // x-y pair
      var xyPair = {
          x: tVal,
          y: message.pose_with_covariance.pose.position.z
      }
      emaData.push(xyPair)
      if (!heightChartPaused && !showingUkfAnalysis) {
          // heightChart.options.scales.xAxes[0].ticks.min = heightChartMinTime;
          // heightChart.options.scales.xAxes[0].ticks.max = heightChartMaxTime;
          heightChart.data.datasets[4].data = emaData.slice();
          // Avoid updating too often, to avoid shaky plotting?
          // heightChart.update();
      }
    });

    stateGroundTruthSub.subscribe(function(message) {
      //printProperties(message);
      currTime = message.header.stamp.secs + message.header.stamp.nsecs/1.0e9;
      if (!gotFirstHeight) {
          gotFirstHeight = true;
          startTime = currTime;
      }
      tVal = currTime - startTime;
      // Have the plot scroll in time, showing a window of windowSize seconds
      if (tVal > windowSize) {
          spanningFullWindow = true;
          // Avoid changing axis limits too often, to avoid shaky plotting?
          // heightChartMinTime = tVal - windowSize;
          // heightChartMaxTime = tVal;

          // Remove first element of array while difference compared to current
          // time is greater than the windowSize
          while (stateGroundTruthData.length > 0 &&
                 (tVal - stateGroundTruthData[0].x > windowSize)) {
              stateGroundTruthData.splice(0, 1);
          }
      }
      // Add new height estimate to end of the data array
      // x-y pair
      var zEstimate = message.pose.position.z;
      if (zEstimate != null) {
          var xyPair = {
              x: tVal,
              y: zEstimate
          }
          stateGroundTruthData.push(xyPair);
      }
      updateGroundTruthXYChart(message);
      if (!heightChartPaused && !showingUkfAnalysis) {
          // heightChart.options.scales.xAxes[0].ticks.min = heightChartMinTime;
          // heightChart.options.scales.xAxes[0].ticks.max = heightChartMaxTime;
          heightChart.data.datasets[5].data = stateGroundTruthData.slice();
          // Avoid updating too often, to avoid shaky plotting?
          // heightChart.update();
      }
    });

    barosub.subscribe(function(message) {
        currTime = new Date().getTime() / 1000;
        if (!gotFirstHeight) {
            gotFirstHeight = true;
            startTime = currTime;
        }
        tVal = currTime - startTime;
        // Have the plot scroll in time, showing a window of windowSize seconds
        if (tVal > windowSize) {
            spanningFullWindow = true;
            heightChartMinTime = tVal - windowSize;
            heightChartMaxTime = tVal;
            // Remove first element of array while difference compared to current
            // time is greater than the windowSize
            while (baroData.length > 0 &&
                   (tVal - baroData[0].x > windowSize)) {
                baroData.splice(0, 1);
            }
        }
        // Add new barometer reading to end of the data array
        // x-y pair
        var xyPair = {
            x: tVal,
            y: message.data
        }
        baroData.push(xyPair)
        if (!heightChartPaused && !showingUkfAnalysis) {
            heightChart.options.scales.xAxes[0].ticks.min = heightChartMinTime;
            heightChart.options.scales.xAxes[0].ticks.max = heightChartMaxTime;
            heightChart.data.datasets[6].data = baroData.slice();
            heightChart.update();
        }
    });

    imageStream();
  }

  function imageStream() {
    var hostname = document.getElementById('hostname').value;
    
    // Устанавливаем обычное изображение
    var image = document.getElementById('cameraImage');
    image.src = "http://" + hostname + ":8080/stream?topic=/raspicam_node/image&quality=70&type=mjpeg";
    
    // Устанавливаем изображение с маркерами
    var markersImage = document.getElementById('markersImage');
    markersImage.src = "http://" + hostname + ":8080/stream?topic=/drone_vision/image_with_marks&quality=70&type=mjpeg";
  }

  // Функция для обновления UI с информацией о маркерах
  function updateMarkersUI(markersData) {
    var markersList = document.getElementById('markers-list');
    var noMarkersMessage = document.getElementById('no-markers-message');
    
    // Очищаем текущий список маркеров
    markersList.innerHTML = '';
    
    if (markersData && markersData.markers && markersData.markers.length > 0) {
        // Есть маркеры для отображения
        noMarkersMessage.style.display = 'none';
        markersList.style.display = 'block';
        
        // Сортируем маркеры по расстоянию (ближайшие сверху)
        markersData.markers.sort(function(a, b) {
            return a.distance - b.distance;
        });
        
        // Добавляем каждый маркер в список
        markersData.markers.forEach(function(marker) {
            var markerItem = document.createElement('div');
            markerItem.className = 'marker-item';
            
            var markerContent = '<div class="marker-id">Маркер #' + marker.id + '</div>';
            markerContent += '<div class="marker-distance">Расстояние: ' + marker.distance.toFixed(2) + ' м</div>';
            markerContent += '<div class="marker-coords">X: ' + marker.x.toFixed(2) + 
                            ' м, Y: ' + marker.y.toFixed(2) + 
                            ' м, Z: ' + marker.z.toFixed(2) + ' м</div>';
            
            markerItem.innerHTML = markerContent;
            markersList.appendChild(markerItem);
        });
        
        // Показываем уведомление о новых маркерах (только при первом обнаружении)
        if (!markersUpdateTimer) {
            showNotification('Обнаружены ArUco маркеры: ' + markersData.markers.length + ' шт.', 'info');
            
            // Устанавливаем таймер, чтобы не показывать уведомления слишком часто
            markersUpdateTimer = setTimeout(function() {
                markersUpdateTimer = null;
            }, 5000);
        }
    } else {
        // Нет маркеров для отображения
        noMarkersMessage.style.display = 'block';
        markersList.style.display = 'none';
    }
  }

  var irAlphaVal = 0;
  var increasingAlpha = true;
  function pulseIr() {
      // Function to pulse the IR background color in the Height chart when
      // paused, to indicate data are coming in
      if (irAlphaVal <= 0.5 && increasingAlpha) {
          irAlphaVal += 0.005
      } else {
          increasingAlpha = false;
          irAlphaVal -= 0.005
      }
      if (irAlphaVal < 0) {
          increasingAlpha = true;
      }

      // Change gradient depending on whether or not the entire chart window
      // is spanned with data
      if (spanningFullWindow) {
          irBackgroundGradient = ctx.createLinearGradient(300, 0, 600, 0);
      } else {
          irBackgroundGradient = ctx.createLinearGradient(0, 0, 600, 0);
      }

      irBackgroundGradient.addColorStop(0, 'rgba(255, 80, 0, 0)');
      irBackgroundGradient.addColorStop(1, 'rgba(255, 80, 0, '+irAlphaVal.toString()+')');
      heightChart.data.datasets[0].backgroundColor = irBackgroundGradient;
      heightChart.data.datasets[0].fill = true;
      heightChart.update();
  }

/*
 * Key event functions
 */

function publishResetTransform() {
  console.log("reset transform");
  resetpub.publish(emptyMsg);
}

function publishToPosition() {
  console.log("to position");
  
  if (!ros || !ros.isConnected) {
    console.log("ROS не подключен. Сначала подключитесь к ROS.");
    return;
  }
  
  if (typeof positionMsg === 'undefined') {
    console.log("positionMsg не инициализирован. Сначала подключитесь к ROS.");
    return;
  }
  
  positionMsg.data = true;
  positionPub.publish(positionMsg);
  updateControlModeUI(true);
}

function publishToVelocity() {
  console.log("to velocity");
  
  if (!ros || !ros.isConnected) {
    console.log("ROS не подключен. Сначала подключитесь к ROS.");
    return;
  }
  
  if (typeof positionMsg === 'undefined') {
    console.log("positionMsg не инициализирован. Сначала подключитесь к ROS.");
    return;
  }
  
  positionMsg.data = false;
  positionPub.publish(positionMsg);
  updateControlModeUI(false);
}

function publishToggleMap() {
    console.log("toggle map")
    mappub.publish(emptyMsg);
}

function publishArm() {
  console.log("arm");
  if (positionMsg.data == true) {
    poseMsg.position.x = 0
    poseMsg.position.y = 0
    poseMsg.position.z = 0
    positionControlPub.publish(poseMsg)
  } else {
    twistMsg.linear.x = 0
    twistMsg.linear.y = 0
    twistMsg.linear.z = 0
    velocityControlPub.publish(twistMsg)
  }
  modeMsg.mode = "ARMED"
  modepub.publish(modeMsg);
}

function publishDisarm() {
  console.log("disarm");
  if (positionMsg.data == true) {
    poseMsg.position.x = 0
    poseMsg.position.y = 0
    poseMsg.position.z = 0
    positionControlPub.publish(poseMsg)
  } else {
    twistMsg.linear.x = 0
    twistMsg.linear.y = 0
    twistMsg.linear.z = 0
    twistMsg.angular.z = 0
    velocityControlPub.publish(twistMsg)
  }
  modeMsg.mode = "DISARMED"
  modepub.publish(modeMsg);
}

function publishTakeoff() {
  console.log("takeoff");
  if (positionMsg.data == true) {
    poseMsg.position.x = 0
    poseMsg.position.y = 0
    poseMsg.position.z = 0
    positionControlPub.publish(poseMsg)
  } else {
    twistMsg.linear.x = 0
    twistMsg.linear.y = 0
    twistMsg.linear.z = 0
    twistMsg.angular.z = 0
    velocityControlPub.publish(twistMsg)
  }
  modeMsg.mode = "FLYING"
  modepub.publish(modeMsg);
}

function publishTranslateLeft() {
  console.log("translate left");
  if (positionMsg.data == true) {
    poseMsg.position.x = -0.1
    poseMsg.position.y = 0
    poseMsg.position.z = 0
    positionControlPub.publish(poseMsg)
  } else {
    twistMsg.linear.x = -0.1
    twistMsg.linear.y = 0
    twistMsg.linear.z = 0
    twistMsg.angular.z = 0
    velocityControlPub.publish(twistMsg)
  }
}

function publishTranslateRight() {
  console.log("translate right");
  if (positionMsg.data == true) {
    poseMsg.position.x = 0.1
    poseMsg.position.y = 0
    poseMsg.position.z = 0
    positionControlPub.publish(poseMsg)
  } else {
    twistMsg.linear.x = 0.1
    twistMsg.linear.y = 0
    twistMsg.linear.z = 0
    twistMsg.angular.z = 0
    velocityControlPub.publish(twistMsg)
  }
}

function publishTranslateForward() {
  console.log("translate forward");
  if (positionMsg.data == true) {
    poseMsg.position.x = 0
    poseMsg.position.y = 0.1
    poseMsg.position.z = 0
    positionControlPub.publish(poseMsg)
  } else {
    twistMsg.linear.x = 0
    twistMsg.linear.y = 0.1
    twistMsg.linear.z = 0
    twistMsg.angular.z = 0
    velocityControlPub.publish(twistMsg)
  }
}

function publishTranslateBackward() {
  console.log("translate backward");
  if (positionMsg.data == true) {
    poseMsg.position.x = 0
    poseMsg.position.y = -0.1
    poseMsg.position.z = 0
    positionControlPub.publish(poseMsg)
  } else {
    twistMsg.linear.x = 0
    twistMsg.linear.y = -0.1
    twistMsg.linear.z = 0
    twistMsg.angular.z = 0
    velocityControlPub.publish(twistMsg)
  }
}

function publishTranslateUp() {
  console.log("translate up");
  poseMsg.position.x = 0
  poseMsg.position.y = 0
  poseMsg.position.z = 0.05
  positionControlPub.publish(poseMsg)
}

function publishTranslateDown() {
  console.log("translate down");
  poseMsg.position.x = 0
  poseMsg.position.y = 0
  poseMsg.position.z = -0.05
  positionControlPub.publish(poseMsg)
}

function publishYawLeft() {
    console.log("yaw left")
    modeMsg.mode = "FLYING"
    twistMsg.linear.x = 0
    twistMsg.linear.y = 0
    twistMsg.linear.z = 0
    twistMsg.angular.z = -50
    velocityControlPub.publish(twistMsg)
}

function publishYawRight() {
    console.log("yaw right")
    modeMsg.mode = "FLYING"
    twistMsg.linear.x = 0
    twistMsg.linear.y = 0
    twistMsg.linear.z = 0
    twistMsg.angular.z = 50
    velocityControlPub.publish(twistMsg)
}

function publishZeroVelocity() {
  console.log("zero velocity");
  
  if (!ros || !ros.isConnected) {
    console.log("ROS не подключен. Сначала подключитесь к ROS.");
    return;
  }
  
  if (typeof twistMsg === 'undefined' || typeof velocityControlPub === 'undefined') {
    console.log("twistMsg или velocityControlPub не инициализированы. Сначала подключитесь к ROS.");
    return;
  }
  
  twistMsg.linear.x = 0
  twistMsg.linear.y = 0
  twistMsg.linear.z = 0
  twistMsg.angular.z = 0
  velocityControlPub.publish(twistMsg)
}

/*
 * Handle IR chart and UKF map
 */


var rawIrDataset = {
  label: 'Сырые данные IR',
  data: Array(0), // initialize array of length 0
  borderWidth: 2,
  pointRadius: 0,
  fill: false,
  borderColor: 'rgba(255, 99, 132, 1)',
  backgroundColor: 'rgba(255, 99, 132, 0.1)',
  lineTension: 0, // remove smoothing
  itemID: 0
};
var rawIrData = Array(0);

var rawVelocityDataset = {
  label: 'Данные скорости',
  data: Array(0), // initialize array of length 0
  borderWidth: 2,
  pointRadius: 0,
  fill: false,
  borderColor: 'rgba(255, 99, 132, 1)',
  backgroundColor: 'rgba(255, 99, 132, 0.1)',
  lineTension: 0, // remove smoothing
  itemID: 0
};
var rawVelocityData = Array(0);

var ukfDataset = {
  label: 'UKF Высота',
  data: Array(0), // initialize array of length 0
  borderWidth: 2,
  pointRadius: 0,
  fill: false,
  borderColor: 'rgba(54, 162, 235, 1)',
  backgroundColor: 'rgba(54, 162, 235, 0.1)',
  lineTension: 0, // remove smoothing
  itemID: 1
}
var ukfData = Array(0);

var ukfPlusSigmaDataset = {
  label: 'UKF +sigma',
  data: Array(0), // initialize array of length 0
  borderWidth: 0,
  pointRadius: 0,
  fill: '+1', // fill to the next dataset
  borderColor: 'rgba(54, 162, 235, 0)', // full transparency
  backgroundColor: 'rgba(54, 162, 235, 0.2)',
  lineTension: 0, // remove smoothing
  itemID: 2
}
var ukfPlusSigmaData = Array(0);

var ukfMinusSigmaDataset = {
  label: 'UKF -sigma',
  data: Array(0), // initialize array of length 0
  borderWidth: 0,
  pointRadius: 0,
  fill: false,
  borderColor: 'rgba(54, 162, 235, 0)', // full transparency
  //backgroundColor: 'rgba(54, 162, 235, 0.1)'
  lineTension: 0, // remove smoothing
  itemID: 3
}
var ukfMinusSigmaData = Array(0);

var emaDataset = {
  label: 'EMA-Сглаженная высота',
  data: Array(0), // initialize array of length 0
  borderWidth: 2,
  pointRadius: 0,
  fill: false,
  borderColor: 'rgba(255, 206, 86, 1)',
  backgroundColor: 'rgba(255, 206, 86, 0.1)',
  lineTension: 0, // remove smoothing
  itemID: 4
}
var emaData = Array(0);

var stateGroundTruthDataset = {
  label: 'Истинная высота',
  data: Array(0), // initialize array of length 0
  borderWidth: 2,
  pointRadius: 0,
  fill: false,
  borderColor: 'rgba(75, 192, 192, 1)',
  backgroundColor: 'rgba(75, 192, 192, 0.1)',
  lineTension: 0, // remove smoothing
  itemID: 5
}
var stateGroundTruthData = Array(0);

//--------------------------------------

var residualDataset = {
  label: 'Ошибка между UKF и реальностью',
  data: Array(0), // initialize array of length 0
  borderWidth: 2,
  pointRadius: 0,
  fill: false,
  borderColor: 'rgba(255, 99, 132, 1)',
  backgroundColor: 'rgba(255, 99, 132, 0.1)',
  lineTension: 0, // remove smoothing
}
var residualData = Array(0);

var plusSigmaDataset = {
  label: '+1 sigma',
  data: Array(0), // initialize array of length 0
  borderWidth: 2,
  pointRadius: 0,
  fill: '+1', // fill to the next dataset
  borderColor: 'rgba(54, 162, 235, 1)',
  backgroundColor: 'rgba(54, 162, 235, 0.2)',
  lineTension: 0, // remove smoothing
}
var plusSigmaData = Array(0);

var minusSigmaDataset = {
  label: '-1 sigma',
  data: Array(0), // initialize array of length 0
  borderWidth: 2,
  pointRadius: 0,
  fill: false,
  borderColor: 'rgba(54, 162, 235, 1)',
  backgroundColor: 'rgba(54, 162, 235, 0.2)',
  lineTension: 0, // remove smoothing
}
var minusSigmaData = Array(0);

var baroDataset = {
  label: 'Данные барометра',
  data: Array(0), // initialize array of length 0
  borderWidth: 2,
  pointRadius: 0,
  fill: false,
  borderColor: 'rgba(75, 192, 192, 1)',
  backgroundColor: 'rgba(75, 192, 192, 0.1)',
  lineTension: 0, // remove smoothing
  itemID: 6
};
var baroData = Array(0);

var ctx;
var xyctx;

function loadHeightChartStandardView() {
    ctx = document.getElementById("heightChart").getContext('2d');
    heightChart = new Chart(ctx, {
        type: 'line',
        data: {
            datasets: [
                rawIrDataset,
                ukfDataset,
                ukfPlusSigmaDataset,
                ukfMinusSigmaDataset,
                emaDataset,
                stateGroundTruthDataset,
                baroDataset
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: {
               duration: 0,
            },
            layout: {
                padding: {
                    left: 10,
                    right: 25,
                    top: 25,
                    bottom: 10
                }
            },
            scales: {
                yAxes: [{
                    ticks: {
                        beginAtZero: true,
                        min: 0,
                        max: 3,
                        stepSize: 0.2,
                        fontColor: '#edf1f7',
                        padding: 10
                    },
                    scaleLabel: {
                        display: true,
                        labelString: 'Высота (метры)',
                        fontColor: '#edf1f7',
                        fontSize: 14,
                        padding: 10
                    },
                    gridLines: {
                        color: 'rgba(255, 255, 255, 0.1)',
                        zeroLineColor: 'rgba(255, 255, 255, 0.25)',
                        drawBorder: true
                    }
                }],
                xAxes: [{
                    type: 'linear',
                    display: false,
                    ticks: {
                        min: 0,
                        max: windowSize,
                        stepSize: windowSize,
                        fontColor: '#edf1f7'
                    },
                    gridLines: {
                        color: 'rgba(255, 255, 255, 0.1)',
                        zeroLineColor: 'rgba(255, 255, 255, 0.25)'
                    }
                }]
            },
            legend: {
                display: true,
                position: 'top',
                labels: {
                    fontColor: '#edf1f7',
                    boxWidth: 15,
                    padding: 15,
                    // Filter out UKF standard deviation datasets and datasets
                    // that have no data in them
                    filter: function(itemInLegend, chartData) {
                        var itemIndex = itemInLegend.datasetIndex;
                        return ((itemIndex != ukfPlusSigmaDataset.itemID &&
                                 itemIndex != ukfMinusSigmaDataset.itemID) &&
                                 (chartData.datasets[itemIndex].data.length != 0));
                    }
                }
            },
            tooltips: {
                backgroundColor: 'rgba(0, 0, 0, 0.7)',
                titleFontColor: '#fff',
                bodyFontColor: '#fff',
                displayColors: false
            }
        }
    });
}

function loadHeightChartUkfAnalysis() {
    ctx = document.getElementById("heightChart").getContext('2d');
    heightChart = new Chart(ctx, {
        type: 'line',
        data: {
            datasets: [
                residualDataset,
                plusSigmaDataset,
                minusSigmaDataset
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: {
               duration: 0,
            },
            layout: {
                padding: {
                    left: 10,
                    right: 25,
                    top: 25,
                    bottom: 10
                }
            },
            scales: {
                yAxes: [{
                    scaleLabel: {
                        display: true,
                        labelString: 'Высота (метры)',
                        fontColor: '#edf1f7',
                        fontSize: 14,
                        padding: 10
                    },
                    ticks: {
                        fontColor: '#edf1f7',
                        padding: 10
                    },
                    gridLines: {
                        color: 'rgba(255, 255, 255, 0.1)',
                        zeroLineColor: 'rgba(255, 255, 255, 0.25)',
                        drawBorder: true
                    }
                }],
                xAxes: [{
                    type: 'linear',
                    display: false,
                    ticks: {
                        min: 0,
                        max: windowSize,
                        stepSize: windowSize,
                        fontColor: '#edf1f7'
                    },
                    gridLines: {
                        color: 'rgba(255, 255, 255, 0.1)',
                        zeroLineColor: 'rgba(255, 255, 255, 0.25)'
                    }
                }]
            },
            legend: {
                display: true,
                position: 'top',
                labels: {
                    fontColor: '#edf1f7',
                    boxWidth: 15,
                    padding: 15
                }
            },
            tooltips: {
                backgroundColor: 'rgba(0, 0, 0, 0.7)',
                titleFontColor: '#fff',
                bodyFontColor: '#fff',
                displayColors: false
            }
        }
    });
}

function loadVelocityChartStandardView() {
    ctx = document.getElementById("velocityChart").getContext('2d');
    velocityChart = new Chart(ctx, {
        type: 'line',
        data: {
            datasets: [
                rawVelocityDataset
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: {
               duration: 0,
            },
            layout: {
                padding: {
                    left: 10,
                    right: 25,
                    top: 25,
                    bottom: 10
                }
            },
            scales: {
                yAxes: [{
                    ticks: {
                        beginAtZero: true,
                        min: 0,
                        max: 3,
                        stepSize: 0.2,
                        fontColor: '#edf1f7',
                        padding: 10
                    },
                    scaleLabel: {
                        display: true,
                        labelString: 'Скорость (м/с)',
                        fontColor: '#edf1f7',
                        fontSize: 14,
                        padding: 10
                    },
                    gridLines: {
                        color: 'rgba(255, 255, 255, 0.1)',
                        zeroLineColor: 'rgba(255, 255, 255, 0.25)',
                        drawBorder: true
                    }
                }],
                xAxes: [{
                    type: 'linear',
                    display: false,
                    ticks: {
                        min: 0,
                        max: windowSize,
                        stepSize: windowSize,
                        fontColor: '#edf1f7'
                    },
                    gridLines: {
                        color: 'rgba(255, 255, 255, 0.1)',
                        zeroLineColor: 'rgba(255, 255, 255, 0.25)'
                    }
                }]
            },
            legend: {
                display: true,
                position: 'top',
                labels: {
                    fontColor: '#edf1f7',
                    boxWidth: 15,
                    padding: 15,
                    // Filter out UKF standard deviation datasets and datasets
                    // that have no data in them
                    filter: function(itemInLegend, chartData) {
                        var itemIndex = itemInLegend.datasetIndex;
                        return ((itemIndex != ukfPlusSigmaDataset.itemID &&
                                 itemIndex != ukfMinusSigmaDataset.itemID) &&
                                 (chartData.datasets[itemIndex].data.length != 0));
                    }
                }
            },
            tooltips: {
                backgroundColor: 'rgba(0, 0, 0, 0.7)',
                titleFontColor: '#fff',
                bodyFontColor: '#fff',
                displayColors: false
            }
        }
    });
}


$(document).ready(function() {
    loadHeightChartStandardView();
    loadVelocityChartStandardView();    
    xyctx = document.getElementById("xyChart").getContext('2d');
    xyChart = new Chart(xyctx, {
        type: 'line',
        data: {
            datasets: [
                {
                  data: Array(0), // initialize array of length 0
                  borderWidth: 2,
                  pointRadius: 0,
                  fill: false,
                  borderColor: 'rgba(153, 102, 255, 1)',
                  backgroundColor: 'rgba(153, 102, 255, 0.1)',
                  lineTension: 0, // remove smoothing
              },
              {
                data: Array(0), // initialize array of length 0
                borderWidth: 2,
                pointRadius: 0,
                fill: false,
                borderColor: 'rgba(153, 102, 255, 1)',
                backgroundColor: 'rgba(153, 102, 255, 0.1)',
                lineTension: 0, // remove smoothing
              },
              {
                data: Array(0), // initialize array of length 0
                borderWidth: 2,
                pointRadius: 0,
                fill: false,
                borderColor: 'rgba(153, 102, 255, 1)',
                backgroundColor: 'rgba(153, 102, 255, 0.1)',
                lineTension: 0, // remove smoothing
              },
              {
                data: Array(0), // initialize array of length 0
                borderWidth: 2,
                pointRadius: 0,
                fill: false,
                borderColor: 'rgba(54, 162, 235, 1)',
                backgroundColor: 'rgba(54, 162, 235, 0.1)',
                lineTension: 0, // remove smoothing
              },
              {
                data: Array(0), // initialize array of length 0
                borderWidth: 2,
                pointRadius: 0,
                fill: false,
                borderColor: 'rgba(54, 162, 235, 1)',
                backgroundColor: 'rgba(54, 162, 235, 0.1)',
                lineTension: 0, // remove smoothing
              },
              {
                data: Array(0), // initialize array of length 0
                borderWidth: 2,
                pointRadius: 0,
                fill: false,
                borderColor: 'rgba(54, 162, 235, 1)',
                backgroundColor: 'rgba(54, 162, 235, 0.1)',
                lineTension: 0, // remove smoothing
              },
              {
                data: Array(0), // initialize array of length 0
                borderWidth: 2,
                pointRadius: 0,
                fill: false,
                borderColor: 'rgba(255, 99, 132, 1)',
                backgroundColor: 'rgba(255, 99, 132, 0.1)',
                lineTension: 0, // remove smoothing
              },
              {
                data: Array(0), // initialize array of length 0
                borderWidth: 2,
                pointRadius: 0,
                fill: false,
                borderColor: 'rgba(255, 99, 132, 1)',
                backgroundColor: 'rgba(255, 99, 132, 0.1)',
                lineTension: 0, // remove smoothing
              },
              {
                data: Array(0), // initialize array of length 0
                borderWidth: 2,
                pointRadius: 0,
                fill: false,
                borderColor: 'rgba(255, 99, 132, 1)',
                backgroundColor: 'rgba(255, 99, 132, 0.1)',
                lineTension: 0, // remove smoothing
              },
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: {
               duration: 0,
            },
            layout: {
                padding: {
                    left: 10,
                    right: 25,
                    top: 25,
                    bottom: 10
                }
            },
            scales: {
                yAxes: [{
                    ticks: {
                        min: -2,
                        max: 2,
                        stepSize: 0.5, // Уменьшаем шаг сетки для более детального отображения
                        fontColor: '#edf1f7',
                        padding: 10
                    },
                    scaleLabel: {
                        display: true,
                        labelString: 'Позиция y (метры)',
                        fontColor: '#edf1f7',
                        fontSize: 14,
                        padding: 10
                    },
                    gridLines: {
                        color: 'rgba(255, 255, 255, 0.1)',
                        zeroLineColor: 'rgba(255, 255, 255, 0.25)',
                        drawBorder: true
                    }
                }],
                xAxes: [{
                    type: 'linear',
                    ticks: {
                        min: -2,
                        max: 2,
                        stepSize: 0.5, // Уменьшаем шаг сетки для более детального отображения
                        display: true,
                        fontColor: '#edf1f7',
                        padding: 10
                    },
                    scaleLabel: {
                        display: true,
                        labelString: 'Позиция x (метры)',
                        fontColor: '#edf1f7',
                        fontSize: 14,
                        padding: 10
                    },
                    gridLines: {
                        color: 'rgba(255, 255, 255, 0.1)',
                        zeroLineColor: 'rgba(255, 255, 255, 0.25)'
                    }
                }]
            },
            legend: {
                display: false
            },
            tooltips: {
                backgroundColor: 'rgba(0, 0, 0, 0.7)',
                titleFontColor: '#fff',
                bodyFontColor: '#fff',
                displayColors: false
            }
        }
    });

    init();
    
    // Добавляем обработчики для кнопок переключения режима
    $('#velocityBtn').on('click', function() {
        publishVelocityMode();
        updateControlModeUI(false);
    });
    
    $('#positionBtn').on('click', function() {
        publishPositionMode();
        updateControlModeUI(true);
    });
});

$(window).on("beforeunload", function(e) {
    closeSession();
});

function changeHeightChartYScaleMin() {
    heightChart.options.scales.yAxes[0].ticks.min = parseFloat(document.getElementById('heightMin').value);
    heightChart.update();
}

function changeHeightChartYScaleMax() {
    heightChart.options.scales.yAxes[0].ticks.max = parseFloat(document.getElementById('heightMax').value);
    heightChart.update();
}

function togglePauseHeightChart(btn) {
    heightChartPaused = !heightChartPaused;
    if (heightChartPaused) {
        $(btn).html('<i class="fas fa-play"></i> Воспроизвести');
        irAlphaVal = 0;
    } else {
        $(btn).html('<i class="fas fa-pause"></i> Пауза');
        heightChart.data.datasets[0].backgroundColor = 'rgba(255, 99, 132, 0)';
        heightChart.data.datasets[0].fill = false;
    }
}

function togglePauseVelocityChart(btn) {
    velocityChartPaused = !velocityChartPaused;
    if (velocityChartPaused) {
        $(btn).html('<i class="fas fa-play"></i> Воспроизвести');
    } else {
        $(btn).html('<i class="fas fa-pause"></i> Пауза');
    }
}

function toggleUkfAnalysis(btn) {
    showingUkfAnalysis = !showingUkfAnalysis;
    if (showingUkfAnalysis) {
        $(btn).html('<i class="fas fa-chart-line"></i> Стандартный вид');
        heightChart.destroy();
        loadHeightChartUkfAnalysis();
    } else {
        $(btn).html('<i class="fas fa-chart-bar"></i> UKF Анализ');
        heightChart.destroy();
        loadHeightChartStandardView();
    }
}

function toggleVelUkfAnalysis(btn) {
    // TODO: Implement UKF analysis for velocity
    console.log('UKF Analysis button pressed for velocity');
    if ($(btn).html().indexOf('UKF Анализ') !== -1) {
        $(btn).html('<i class="fas fa-chart-line"></i> Стандартный вид');
    } else {
        $(btn).html('<i class="fas fa-chart-bar"></i> UKF Анализ');
    }
}

function togglePauseXYChart(btn) {
    // Implement XY chart pausing
    console.log('Pause button pressed');
    if ($(btn).html().indexOf('Пауза') !== -1) {
        $(btn).html('<i class="fas fa-play"></i> Воспроизвести');
    } else {
        $(btn).html('<i class="fas fa-pause"></i> Пауза');
    }
}

function publishVelocityMode() {
    if (!ros || !ros.isConnected) {
        console.log("ROS не подключен. Сначала подключитесь к ROS.");
        return;
    }
    
    if (typeof positionMsg === 'undefined') {
        console.log("positionMsg не инициализирован. Сначала подключитесь к ROS.");
        return;
    }
    
    positionMsg.data = false;
    positionPub.publish(positionMsg);
    updateControlModeUI(false);
}

function publishPositionMode() {
    if (!ros || !ros.isConnected) {
        console.log("ROS не подключен. Сначала подключитесь к ROS.");
        return;
    }
    
    if (typeof positionMsg === 'undefined') {
        console.log("positionMsg не инициализирован. Сначала подключитесь к ROS.");
        return;
    }
    
    positionMsg.data = true;
    positionPub.publish(positionMsg);
    updateControlModeUI(true);
}

// Обновляем функцию updateControlModeUI для предотвращения множественных переключений
function updateControlModeUI(isPositionMode) {
    // Проверяем, изменился ли режим с момента последнего обновления
    if (lastPositionMode === isPositionMode) {
        return; // Пропускаем обновление, если режим не изменился
    }
    
    // Сбрасываем предыдущий таймер дебаунса, если он существует
    if (positionModeDebounceTimer) {
        clearTimeout(positionModeDebounceTimer);
    }
    
    // Устанавливаем таймер дебаунса
    positionModeDebounceTimer = setTimeout(function() {
        // Обновляем отслеживание последнего режима
        lastPositionMode = isPositionMode;
        
        if (isPositionMode) {
            // Обновляем кнопки
            $('#velocityBtn').removeClass('active btn-velocity').addClass('btn-secondary');
            $('#positionBtn').removeClass('btn-secondary').addClass('active btn-position');
            
            // Обновляем индикатор режима
            $('#position_state').html('<div class="alert alert-position"><span class="mode-indicator mode-position"></span><strong>РЕЖИМ ПОЗИЦИОНИРОВАНИЯ</strong> <i class="fas fa-map-marker-alt ml-2"></i></div>');
            
            // Обновляем стили инпутов
            $('.control-input').removeClass('velocity').addClass('position');
            
            // Обновляем уведомление для XY Chart
            $('#xy_chart_info').removeClass('alert-warning').addClass('alert-info')
                .html('<i class="fas fa-info-circle"></i> Данные о позиции активны. Карта обновляется.');
                
            // Показываем уведомление
            showNotification('Включен режим позиционирования', 'success');
        } else {
            // Обновляем кнопки
            $('#positionBtn').removeClass('active btn-position').addClass('btn-secondary');
            $('#velocityBtn').removeClass('btn-secondary').addClass('active btn-velocity');
            
            // Обновляем индикатор режима
            $('#position_state').html('<div class="alert alert-velocity"><span class="mode-indicator mode-velocity"></span><strong>РЕЖИМ УПРАВЛЕНИЯ СКОРОСТЬЮ</strong> <i class="fas fa-tachometer-alt ml-2"></i></div>');
            
            // Обновляем стили инпутов
            $('.control-input').removeClass('position').addClass('velocity');
            
            // Обновляем уведомление для XY Chart
            $('#xy_chart_info').removeClass('alert-info').addClass('alert-warning')
                .html('<i class="fas fa-exclamation-triangle"></i> Внимание: Данные о позиции недоступны в режиме скорости.');
                
            // Показываем уведомление
            showNotification('Включен режим управления скоростью', 'info');
        }
    }, notificationDebounceTime);
}

function setControls () {
    if (!ros || !ros.isConnected) {
        console.log("ROS не подключен. Сначала подключитесь к ROS.");
        $('#position_state').html('<div class="alert alert-danger">Ошибка: ROS не подключен</div>');
        return;
    }
    
    if (typeof positionMsg === 'undefined' || typeof poseMsg === 'undefined' || typeof twistMsg === 'undefined') {
        console.log("Сообщения не инициализированы. Сначала подключитесь к ROS.");
        $('#position_state').html('<div class="alert alert-danger">Ошибка: Сообщения не инициализированы</div>');
        return;
    }
    
    x = document.getElementById("controlX").value;
    y = document.getElementById("controlY").value;
    z = document.getElementById("controlZ").value;

    if (positionMsg.data == true) {
        poseMsg.position.x = Number(parseFloat(x));
        poseMsg.position.y = Number(parseFloat(y));
        poseMsg.position.z = Number(parseFloat(z));
        positionControlPub.publish(poseMsg);
    } else {
        twistMsg.linear.x = Number(parseFloat(x));
        twistMsg.linear.y = Number(parseFloat(y));
        twistMsg.linear.z = Number(parseFloat(z));
        velocityControlPub.publish(twistMsg);
    }
}

/*
 * Listen for key events
*/

$(document).keyup(function(event){
  var char = String.fromCharCode(event.which || event.keyCode);
  if (char == "J" || char == "L" || char == "K" || char == "I" || char == "W" || char == "S" || char == "A" || char == "D") {
    publishZeroVelocity();
  }
});

$(document).keypress(function(event){
  var char = String.fromCharCode(event.which || event.keyCode);
  if (char == ';') {
    publishArm();
  } else if (char == ' ') {
    publishDisarm();
  } else if (char == 'r') {
    publishResetTransform();
  } else if (char == 't') {
    publishTakeoff();
  } else if (char == 'p') {
    publishToPosition();
  } else if (char == 'v') {
    publishToVelocity();
  } else if (char == 'm') {
    publishToggleMap();
  }
});

$(document).keydown(function(event){
  var char = String.fromCharCode(event.which || event.keyCode);
  // console.log("Key down: " + char);
  if (char == 'J') {
    publishTranslateLeft();
  } else if (char == 'L') {
    publishTranslateRight();
  } else if (char == "K") {
    publishTranslateBackward();
  } else if (char == "I") {
    publishTranslateForward();
  } else if (char == "W") {
    publishTranslateUp();
  } else if (char == "S") {
    publishTranslateDown();
  } else if (char == "A") {
    publishYawLeft();
  } else if (char == "D") {
    publishYawRight();
  } else {
    //console.log('undefined key: ' + event.keyCode);
  }
});

// Добавляем функцию init() в конец файла
function init() {
  // Инициализация интерфейса
  console.log("Инициализация интерфейса");
  
  // Проверяем, подключены ли мы к ROS
  if (ros && ros.isConnected) {
    // Определяем текущий режим и обновляем UI
    var isPositionMode = positionMsg && positionMsg.data;
    updateControlModeUI(isPositionMode);
    // Обновляем состояние кнопок подключения
    toggleConnectionButtons(true);
  } else {
    // Если не подключены, показываем соответствующее сообщение
    $('#position_state').html('<div class="alert alert-warning"><i class="fas fa-exclamation-triangle mr-2"></i>Подключитесь к ROS для управления</div>');
    
    // Устанавливаем стили по умолчанию для кнопок и инпутов
    $('#velocityBtn').addClass('btn-velocity');
    $('.control-input').addClass('velocity');
    
    // Устанавливаем уведомление по умолчанию для XY Chart
    $('#xy_chart_info').removeClass('alert-info').addClass('alert-warning')
        .html('<i class="fas fa-exclamation-triangle"></i> Подключитесь к ROS для отображения данных о позиции.');
    
    // Обновляем состояние кнопок подключения
    toggleConnectionButtons(false);
  }
}

// Функция для переключения видимости кнопок подключения/отключения
function toggleConnectionButtons(isConnected) {
    if (isConnected) {
        $('#connectButton').hide();
        $('#disconnectButton').show();
        $('#hostname').prop('disabled', true).addClass('disabled-input');
    } else {
        $('#connectButton').show();
        $('#disconnectButton').hide();
        $('#hostname').prop('disabled', false).removeClass('disabled-input');
    }
}

// Подписка на состояние дрона
function setupStateSubscription() {
    if (!ros || !ros.isConnected) {
        console.log("ROS не подключен. Невозможно настроить подписку на состояние.");
        return;
    }
    
    var stateSub = new ROSLIB.Topic({
        ros: ros,
        name: '/pidrone/state/ema',
        messageType: 'pidrone_pkg/State',
        queue_length: 1,
        throttle_rate: 100
    });
    
    stateSub.subscribe(function(message) {
        // Обновление позиции
        document.getElementById('drone_x').textContent = message.pose_with_covariance.pose.position.x.toFixed(2);
        document.getElementById('drone_y').textContent = message.pose_with_covariance.pose.position.y.toFixed(2);
        document.getElementById('drone_z').textContent = message.pose_with_covariance.pose.position.z.toFixed(2);
        
        // Обновление скорости
        document.getElementById('drone_vx').textContent = message.twist_with_covariance.twist.linear.x.toFixed(2);
        document.getElementById('drone_vy').textContent = message.twist_with_covariance.twist.linear.y.toFixed(2);
        document.getElementById('drone_vz').textContent = message.twist_with_covariance.twist.linear.z.toFixed(2);
        
        // Обновление ориентации (преобразование кватерниона в углы Эйлера)
        var quaternion = message.pose_with_covariance.pose.orientation;
        var euler = quaternionToEuler(quaternion);
        document.getElementById('drone_yaw').textContent = (euler.yaw * 180 / Math.PI).toFixed(2);
        
        // Обновление прогресс-баров
        updateProgressBars(message);
    });
}

// Преобразование кватерниона в углы Эйлера
function quaternionToEuler(q) {
    return {
        roll: Math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
        pitch: Math.asin(2 * (q.w * q.y - q.z * q.x)),
        yaw: Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    };
}

// Обновление прогресс-баров
function updateProgressBars(message) {
    // Нормализация значений для прогресс-баров
    var xPercent = ((message.pose_with_covariance.pose.position.x + 1) / 2 * 100).toFixed(0);
    document.getElementById('drone_x_bar').style.width = xPercent + '%';
    
    var yPercent = ((message.pose_with_covariance.pose.position.y + 1) / 2 * 100).toFixed(0);
    document.getElementById('drone_y_bar').style.width = yPercent + '%';
    
    var zPercent = (message.pose_with_covariance.pose.position.z / 3 * 100).toFixed(0);
    document.getElementById('drone_z_bar').style.width = zPercent + '%';
    
    var vxPercent = ((message.twist_with_covariance.twist.linear.x + 1) / 2 * 100).toFixed(0);
    document.getElementById('drone_vx_bar').style.width = vxPercent + '%';
    
    var vyPercent = ((message.twist_with_covariance.twist.linear.y + 1) / 2 * 100).toFixed(0);
    document.getElementById('drone_vy_bar').style.width = vyPercent + '%';
    
    var vzPercent = ((message.twist_with_covariance.twist.linear.z + 1) / 2 * 100).toFixed(0);
    document.getElementById('drone_vz_bar').style.width = vzPercent + '%';
    
    // Получаем углы Эйлера из кватерниона
    var euler = quaternionToEuler(message.pose_with_covariance.pose.orientation);
    var yawPercent = (((euler.yaw * 180 / Math.PI) + 180) / 360 * 100).toFixed(0);
    document.getElementById('drone_yaw_bar').style.width = yawPercent + '%';
}

// Улучшаем функцию showNotification для предотвращения дублирования
var lastNotification = {
    message: '',
    type: '',
    timestamp: 0
};

function showNotification(message, type) {
    var currentTime = new Date().getTime();
    
    // Проверяем, не было ли такого же уведомления недавно
    if (message === lastNotification.message && 
        type === lastNotification.type &&
        currentTime - lastNotification.timestamp < 2000) {
        return; // Пропускаем дублирующееся уведомление
    }
    
    // Обновляем информацию о последнем уведомлении
    lastNotification.message = message;
    lastNotification.type = type;
    lastNotification.timestamp = currentTime;
    
    // Создаем элемент уведомления
    var notification = $('<div class="notification notification-' + type + '">' +
                         '<div class="notification-icon"><i class="fas ' + getIconForType(type) + '"></i></div>' +
                         '<div class="notification-content">' + message + '</div>' +
                         '<div class="notification-close"><i class="fas fa-times"></i></div>' +
                         '</div>');
    
    // Добавляем на страницу
    $('#notification-container').append(notification);
    
    // Анимируем появление
    setTimeout(function() {
        notification.addClass('show');
    }, 10);
    
    // Автоматически скрываем через 5 секунд
    setTimeout(function() {
        notification.removeClass('show');
        setTimeout(function() {
            notification.remove();
        }, 300);
    }, 5000);
    
    // Обработчик клика по кнопке закрытия
    notification.find('.notification-close').on('click', function() {
        notification.removeClass('show');
        setTimeout(function() {
            notification.remove();
        }, 300);
    });
}

// Функция для определения иконки в зависимости от типа уведомления
function getIconForType(type) {
    switch(type) {
        case 'success':
            return 'fa-check-circle';
        case 'warning':
            return 'fa-exclamation-triangle';
        case 'error':
            return 'fa-exclamation-circle';
        case 'info':
        default:
            return 'fa-info-circle';
    }
}

// Функция для настройки подписки на информацию о маркерах
function setupMarkersSubscription() {
    if (!ros || !ros.isConnected) {
        console.log("ROS не подключен. Невозможно настроить подписку на маркеры.");
        return;
    }
    
    markersInfoSub = new ROSLIB.Topic({
        ros: ros,
        name: '/drone_vision/markers_info',
        messageType: 'std_msgs/String',
        queue_length: 1
    });
    
    markersInfoSub.subscribe(function(message) {
        try {
            var markersData = JSON.parse(message.data);
            lastMarkersInfo = markersData;
            
            // Обновляем UI с информацией о маркерах
            updateMarkersUI(markersData);
        } catch (e) {
            console.error('Ошибка при обработке данных о маркерах:', e);
        }
    });
    
    console.log("Подписка на информацию о маркерах настроена");
}
