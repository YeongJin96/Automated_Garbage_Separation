# Automated_Garbage_Separation
YOLOv5s와 ROS를 이용한 Robotics Arms으로 자동으로 쓰레기를 분리수거합니다.
해당 프로젝트는 컨베이어 벨트를 지나는 쓰레기를 자동으로 분리수거를 하는 것이 목표입니다.



### 하드웨어

먼저 컨베이어 벨트위의 쓰레기를 집어낼 도구로 로봇 팔을 선택하였고, 로봇 팔을 제어할 프로세서는 Jetson nano(4GB)를 사용하였습니다.



![Jetson_nano&Robotic-arm](C:\Users\dydyz\OneDrive\바탕 화면\GitHub_blog\YeongJin96.github.io\assets\img\jetsonnano_with_robot_arm.jpg)

![Jetson_nano](C:\Users\dydyz\OneDrive\바탕 화면\GitHub_blog\YeongJin96.github.io\assets\img\KakaoTalk_20221214_215040052.png)



로봇 팔은 물건을 집을 수 있는 집게와 이미지 인식을 위한 카메라를 포함하고 있었고, Jetson nano와의 호환성을 고려해 선택하게 되었습니다.

---

### 데이터

데이터셋은 PET, Plastic, Can, Liquorbottle, Glass, Metal, Vinyl, Paper, Yogurt로 총 9개의 레이블을 가지고있습니다.

모델을 학습할 때 3가지 종류의 데이터를 사용했습니다.

1. 동영상으로 동작중인 컨베이어 벨트위에 쓰레기들을 올려 촬영하고, 1초단위로 프레임을 끊어 라벨링한 데이터 (1개의 이미지에 많은 object가 존재)
2. 아파트 분리수거장에서 하나씩 쓰레기를 꺼내 촬영하고, 라벨링한 데이터 (1개의 이미지에 단일 object)
3. Ai Hub사이트의 라벨링된 쓰레기 데이터 (1개의 이미지에 단일 object와 다수 object가 섞여있음)



---



### 인공지능 모델

로봇 팔이 스스로 쓰레기를 감지하고 분리수거하기 위해 인공지능을 탑재했습니다.

쓰레기는 컨베이어 벨트위에 놓여있어서 실시간으로 이미지를 처리해야 했기 때문에 Object detection을 사용했습니다.

Jetson nano의 메모리와 VRAM은 4GB로 비교적 낮은 컴퓨팅 파워를 가지고있어 Object detection모델 중 비교적 가벼운 모델인 YOLOv5s를 사용했습니다. (사진의 가장 우측모델)

![YOLO 버전별 소요 메모리](C:\Users\dydyz\OneDrive\바탕 화면\GitHub_blog\YeongJin96.github.io\assets\img\image01.png)

학습결과

![image05](C:\Users\dydyz\OneDrive\바탕 화면\GitHub_blog\YeongJin96.github.io\assets\img\image05.png)

---



### ROS

처음에는 Jetson nano에 YOLO모델을 탑재하고 Jetson nano에서 직접적으로 Object Detection을 하려했습니다.

하지만, Jetson nano에서 YOLO를 구동했을 때 몇가지 문제점이 있었습니다.

1. 메모리 문제

   ![nano_memory](C:\Users\dydyz\OneDrive\바탕 화면\GitHub_blog\YeongJin96.github.io\assets\img\KakaoTalk_20221215_202309730.png)

   (Jenson nano에서 직접 YOLO모델 작동시 메모리 스왑을 했음에도 메모리가 부족했음)

2. 추론 속도 문제

   ![image11](C:\Users\dydyz\OneDrive\바탕 화면\GitHub_blog\YeongJin96.github.io\assets\img\image11.png)

   (27fps를 가진 4분 44초 영상에 대한 추론을 Jetson nano에서 직접 처리했을 때 초당 약 7개의 프레임밖에 처리하지 못했음)

   ![image12](C:\Users\dydyz\OneDrive\바탕 화면\GitHub_blog\YeongJin96.github.io\assets\img\image12.png)

   (같은 조건으로 서버에서 영상을 추론했을 때 초당 약 344개의 프레임을 처리함)

3. 앞선 문제들 때문에 로봇팔 제어를 위한 추가적인 프로그램 구동이 어려움



해당 문제들 때문에, Jetson nano에서 직접적으로 YOLO모델을 탑재해 추론할 경우 매우 느린 반응속도를 보여주었습니다. 때문에 ROS를 이용해 서버와 로봇 팔을 publisher와 subscriber로 나눈 뒤, 서버에서 Object Detection을 처리해 탐지된 object의 정보와 위치를 파악하고 로봇팔이 움직여야 할 정보들을 Jetson nano에게 쏴주고, Jetson nano에서는 받은 정보를 토대로 로봇 팔만 제어하도록 하였습니다.

![image13](C:\Users\dydyz\OneDrive\바탕 화면\GitHub_blog\YeongJin96.github.io\assets\img\image13.png)

쓰레기 분리수거를 위해 사용한 노드들은 다음과 같습니다.

1. Jetson nano의 로봇 팔에 장착된 카메라로 부터 촬영한 영상을 보내는 노드

   로봇 팔의 카메라로 찍은 cv이미지를 서버에 보내기 위해 Message로 변환하고 토픽으로 퍼블리싱

   ```c++
   msg = cv_image->toImageMsg();
       if (cam_info_msg.distortion_model == ""){
           NODELET_WARN_STREAM("No calibration file given, publishing a reasonable default camera info.");
           cam_info_msg = get_default_camera_info_from_image(msg);
       }
       pub.publish(*msg, cam_info_msg, ros::Time::now());
   ```

2. 서버에서 카메라 이미지들을 받는 노드

   토픽으로 받은 이미지들을 받을 때 queue_size는 1로 설정 (실시간성을 위해)

   ```python
   input_image_type, input_image_topic, _ = get_topic_type(rospy.get_param("~input_image_topic"), blocking = True)
   self.compressed_input = input_image_type == "sensor_msgs/CompressedImage"
   
   if self.compressed_input:
       self.image_sub = rospy.Subscriber(
           input_image_topic, CompressedImage, self.callback, queue_size=1
       )
   else:
       self.image_sub = rospy.Subscriber(
           input_image_topic, Image, self.callback, queue_size=1
       )
   ```

3. 이미지의 object들을 탐지하고, object의 정보를 생성하는 노드 (YOLO모델 추론)

   이미지를 추론하고, 검출된 object의 bounding_box정보를 생성하고 로봇 팔을 제어할 노드에 퍼블리싱

   ```python
       # Write results
       for *xyxy, conf, cls in reversed(det):
           bounding_box = BoundingBox()
           c = int(cls)
           # Fill in bounding box message
           bounding_box.Class = self.names[c]
           bounding_box.probability = conf 
           bounding_box.xmin = int(xyxy[0])
           bounding_box.ymin = int(xyxy[1])
           bounding_box.xmax = int(xyxy[2])
           bounding_box.ymax = int(xyxy[3])
   
           bounding_boxes.bounding_boxes.append(bounding_box)
   
           # Annotate the image
           if self.publish_image or self.view_image:  # Add bbox to image
                 # integer class
               label = f"{self.names[c]} {conf:.2f}"
               annotator.box_label(xyxy, label, color=colors(c, True))
   
   
           ### POPULATE THE DETECTION MESSAGE HERE
   
       # Stream results
       im0 = annotator.result()
   
   # Publish prediction
   self.pred_pub.publish(bounding_boxes)
   ```

4. bounding_box의 정보를 받고 정보를 토대로 로봇 팔 제어의 명령을 지시하는 노드

   로봇 팔의 관절은 총 6개로 구성되어 있으며, 카메라 영상의 중심을 기준으로 진행했다.

   ```python
   rospy.init_node("pub_test", anonymous=True)
   
   #message
   self.joint_state = JointState()
   self.joint_state.name = ['joint1','joint2','joint3','joint4','joint5','joint6']
   self.ready1 = [-1.27, -0.08, -0.72, -1.32, 0.0, -1.50]
   self.joint_state.position = copy.deepcopy(self.ready1)
   self.pub = rospy.Publisher('move_arm', JointState, queue_size=1)
   self.pub.publish(self.joint_state)
   sleep(2.0)
   
   #follow
   resolution_x = 640
   resolution_y = 480
   threshold = 50
   cam_centerX = resolution_x/2
   cam_centerY = resolution_y/2
   
   self.minX = cam_centerX - threshold
   self.maxX = cam_centerX + threshold
   self.minY = cam_centerY - threshold
   self.maxY = cam_centerY + threshold
   self.speed = 0.01
   self.flag = True
   
   #etc
   self.start_time = time.time()
   
   #yolov5
   rospy.Subscriber("/yolov5/detections", BoundingBoxes, self.my_callback, queue_size=1)
   ```

   로봇 팔이 bounding_box를 따라가도록 하는 코드

   0번 관절과 3,4번째 관절을 사용해 object follow를 진행하였으나, 3,4번째 관절을 함께 제어할 경우 컨베이어 벨트위에 놓인 쓰레기의 위치가 많이 달라질 경우 쓰레기를 집는데 문제가 있다고 판단되어 시연할 때는 사용하지 않았음

   ```python
   #joint 0
   if self.joint_state.position[0] > -1.50 and self.joint_state.position[0] < 1.50:
       if self.minX>b_x:
           self.joint_state.position[0] += (self.speed*2)
       # elif self.maxX<b_x:
       #     self.joint_state.position[0] -= (self.speed/2)
       else: pass
   else: print("joint0 value is maximum, so you don't move joint0")
   
   # #joint 3, 4
   # if self.joint_state.position[2] > -1.50 and self.joint_state.position[2] < 1.50:
   #     if self.maxY<b_y:
   #         self.joint_state.position[2] -= self.speed * 3
   #     # elif self.minY>b_y:
   #         # self.joint_state.position[2] += self.speed
   #     else: pass
   # elif self.joint_state.position[3] > -1.50 and self.joint_state.position[3] < 1.50:
   #     if self.maxY<b_y:
   #         self.joint_state.position[3] -= self.speed * 3
   #     # elif self.minY>b_y:
   #         # self.joint_state.position[3] += self.speed
   #     else: pass
   # else: print("joint3,4 value is maximum, so you don't move joint3,4")
   ```

   [Object_Follow Video](https://www.youtube.com/shorts/hEkUC_udDC4)

   <style>.embed-container { position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; } .embed-container iframe, .embed-container object, .embed-container embed { position: absolute; top: 0; left: 0; width: 100%; height: 100%; }</style><div class='embed-container'><iframe src='https://www.youtube.com/embed/undefined' frameborder='0' allowfullscreen></iframe></div>

   로봇 팔이 object를 따라가다가 집게로 집는 코드
   
   ```python
   if self.joint_state.position[0] < -1.2:
       self.joint_state.position[0] = -1.2
   self.joint_state.position[4] = 3.11
   self.pub.publish(self.joint_state)
   sleep(0.5)
   self.joint_state.position[:4] = [self.joint_state.position[0]+0.15,-1.10, -0.83, -0.14]
   self.pub.publish(self.joint_state)
   sleep(0.5)
   self.joint_state.position[5] = 1.3
   self.pub.publish(self.joint_state)
   
   self.flag = True
   ```

   집게로 object를 집고, 옮기는 코드
   
   ```python
   self.joint_state.position[:4] = [-1.55, -0.17, -0.62, -0.90]
   self.pub.publish(self.joint_state)
   sleep(0.5)
   self.joint_state.position[5] = -1.5
   self.pub.publish(self.joint_state)
   sleep(1.5)
   self.joint_state.position = copy.deepcopy(self.ready1)
   self.pub.publish(self.joint_state)
   sleep(1.0)
   ```

   [Object_Follow+Grab Video](https://www.youtube.com/shorts/FKs7XVyLdyA)
   
   callback 함수
   
   ```python
   def my_callback(self, msg):
           self.pub = rospy.Publisher('move_arm', JointState, queue_size=1)
           self.pub.publish(self.joint_state)
           if msg.bounding_boxes != []:
               ob_class = str(msg.bounding_boxes[0].Class)
               ob_proba = float(msg.bounding_boxes[0].probability)
               ob_xmin = int(msg.bounding_boxes[0].xmin)
               ob_ymin = int(msg.bounding_boxes[0].ymin)
               ob_xmax = int(msg.bounding_boxes[0].xmax)
               ob_ymax = int(msg.bounding_boxes[0].ymax)
   
               bbox_Xcenter = ob_xmin + (ob_xmax - ob_xmin)/2
               bbox_Ycenter = (ob_ymin + (ob_ymax - ob_ymin)/2) + 50
               
               if self.flag:
                   self.firstX = copy.deepcopy(bbox_Xcenter)
                   self.flag = False
   
               if ob_class == 'plastic' or ob_class == 'pet':
               	self.follow_object(bbox_Xcenter, bbox_Ycenter) #grab
                   
               
               if ob_ymax > 475 and ob_xmin < 420:#grab
                   self.grap(self.firstX)
                   self.move_object() 
                   
               # if self.joint_state.position[0] > -0.9:#push
               #     self.push_object()
               
               # if ob_class == 'paper':
               #      self.move_arm() #push
               #     self.follow_object(bbox_Xcenter, bbox_Ycenter) #grab
               #     if ob_ymax > 460:
               #         self.grap(self.firstX)
               #         self.move_object()        
   
               ob_class, ob_ymax = None, None
               self.start_time = time.time()
               
           else: 
               self.flag = True
               current_time = time.time()
               end_time = current_time - self.start_time
               if end_time > 5:
                   print("didn't find object!")
                   self.joint_state.position = copy.deepcopy(self.ready1)
                   self.pub.publish(self.joint_state)
                   sleep(2.0)
   ```
   
   [outcome_video](https://www.youtube.com/watch?v=9_GKYOLCQCs)
