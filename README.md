# move
<p>Modify from : https://github.com/ros-teleop/teleop_twist_keyboard</p>

```Python
moveBindings = {
        'i':(1,0,0,0),   # Forward
        'I':(1,0,0,0),   # Forward
        ',':(-1,0,0,0),  # backward
        '<':(-1,0,0,0),  # backward
        '.':(-1,-1,0,0), # Diagonal bottom right
        '>':(-1,-1,0,0), # Diagonal bottom right
        'j':(0,0,0,1),   # Turn left
        'l':(0,0,0,-1),  # Turn right
        'J':(0,1,0,0),   # Side left
        'L':(0,-1,0,0),  # Side right
        'M':(-1,1,0,0),  # Diagonal bottom left
        'm':(-1,1,0,0),  # Diagonal bottom left
        'U':(1,1,0,0),   # Diagonal top left
        'u':(1,1,0,0),   # Diagonal top left
        'O':(1,-1,0,0),  # Diagonal top right
        'o':(1,-1,0,0),  # Diagonal top right
    }
```
<p align="center">
  <img src="image/1.png" />
</p>
<p align="center">
  <img src="image/2.png" />
</p>

## $\color[RGB]{0,240,43}Skid$ $\color[RGB]{0,240,43}Steer$ $\color[RGB]{0,240,43}/$ $\color[RGB]{0,240,43}Differential$ $\color[RGB]{0,240,43}Drive$
<p>Here is some math for 2 and 4 wheel differential drive vehicles, 2 wheels and a castor, or skid steer tracked vehicles.</p>

<p>$\color[rgb]{1,0,1}Arc$ $\color[rgb]{1,0,1}based$ $\color[rgb]{1,0,1}commands$</p>

<p>The basic skid steer equations are:</p>

>velocity_right = w(RADIUS_OF_ARC_TO_DRIVE + WHEEL_BASE/2)
>
>velocity_left = w(RADIUS_OF_ARC_TO_DRIVE – WHEEL_BASE/2)

<p>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;" Where w is the angular rotation, RADIUS_OF_ARC_TO_DRIVE is the arc radius that the robot should drive, and the WHEEL_BASE is the distance from the center of the left wheel to the center of the right wheel (See image above). (โดยที่wคือการหมุนเชิงมุม RADIUS_OF_ARC_TO_DRIVE คือรัศมีส่วนโค้งที่หุ่นยนต์ควรขับเคลื่อน และ WHEEL_BASE คือระยะทางจากศูนย์กลางของล้อซ้ายไปยังศูนย์กลางของล้อขวา) " ดูจากภาพด้านล่าง</p>
<p align="center">
  <img src="https://www.robotsforroboticists.com/wp-content/uploads/2016/06/Drive_Kinematics-1-302x180.jpg">
</p>
<p>This can also be written as:</p>

>w = (velocity_right-velocity_left)/WHEEL_BASE
<p>There are two special cases:</p>

>if velocity_right == velocity_left :
THEN the radius of the arc is infinite so the robot will drive straight.
>
>if velocity_right == -velocity_left :
THEN the radius of the arc is 0, and the robot rotates in place (ie. point turn)

<p>$\color[rgb]{1,0,1}Linear$ $\color[rgb]{1,0,1}and$ $\color[rgb]{1,0,1}Angular$ $\color[rgb]{1,0,1}Velocity$ $\color[rgb]{1,0,1}Commands$ $\color[rgb]{1,0,1}for$ $\color[rgb]{1,0,1}ROS$</p>
<p>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;" In ROS if using the Twist topic (which is the default for drive messages) (message name is often cmd_vel) you will often set linear_velocity in the linear.x field and angular_velocity in the angular.z field. (ใน ROS หากใช้หัวข้อ Twist (ซึ่งเป็นค่าเริ่มต้นสำหรับข้อความ) (ชื่อข้อความมักจะเป็น cmd_vel) ส่วนใหญ่มักจะตั้งค่า linear_velocity ในช่องlinear.x และ angular_velocity ในช่อง angular.z) "</p>

>velocity_left_cmd = (linear_velocity – angular_velocity * WHEEL_BASE / 2.0)/WHEEL_RADIUS;
>
>velocity_right_cmd = (linear_velocity + angular_velocity * WHEEL_BASE / 2.0)/WHEEL_RADIUS;

## $\color[RGB]{0,240,43}Mecanum$ $\color[RGB]{0,240,43}Wheel$ $\color[RGB]{0,240,43}Math$
<p>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;" In ROS if using the Twist message you will often set the linear.x, linear.y and angular.z fields.
One unrelated note is that if you are operating on uneven terrain then doing mecanum type motions will fail and have a lot of slip. Skid steer type motions will often work better (using the mecanum wheels). (ใน ROS หากใช้ข้อความ Twist คุณมักจะตั้งค่าฟิลด์ linear.x, linear.y และ angular.z
ข้อควรทราบอย่างหนึ่งคือ หากคุณใช้งานในพื้นที่ไม่เรียบ การเคลื่อนไหวแบบ mecanum จะล้มเหลวและมีการลื่นไถลมาก การเคลื่อนไหวแบบลื่นไถลมักจะทำงานได้ดีขึ้น (โดยใช้ล้อ mecanum)) "</p>

>WHEEL_SEPARATION_WIDTH = DISTANCE_LEFT_TO_RIGHT_WHEEL / 2
>
>WHEEL_SEPARATION_LENGTH = DISTANCE_FRONT_TO_REAR_WHEEL / 2
<p>$\color[rgb]{1,0,1}Forward$ $\color[rgb]{1,0,1}Kinematics$</p>
<p><b>(Technically inverse kinematics, but with ground robots often refereed to as forward kinematics)</b></p>
<p>Wheel commands units are in rad/s</p>

>wheel_front_left = (1/WHEEL_RADIUS) * (linear.x – linear.y – (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angular.z);
>
>wheel_front_right = (1/WHEEL_RADIUS) * (linear.x + linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angular.z);
>
>wheel_rear_left = (1/WHEEL_RADIUS) * (linear.x + linear.y – (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angular.z);
>
>wheel_rear_right = (1/WHEEL_RADIUS) * (linear.x – linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angular.z);

<p>To drive a robot you will probably need to also invert one side since the motors are mounted opposite the other side. For example:</p>

>wheel_front_right = -1 * wheel_front_right
>
>wheel_rear_right = -1 * wheel_rear_right

<p>Also this gives an output in rad/s. If your motor controller is operating with encoder counts as the unit you will need to convert the units. (นอกจากนี้ยังให้เอาต์พุตเป็น rad/s หากตัวควบคุมมอเตอร์ของคุณทำงานกับเอ็นโค้ดเดอร์ให้นับเป็น หน่วย เเละคุณจะต้องทำการแปลงหน่วยด้วย)</p>
<p>$\color[rgb]{1,0,1}Inverse$ $\color[rgb]{1,0,1}Kinematics$</p>
<p><b>(Technically forward kinematics, but with ground robots often refereed to as inverse kinematics)</b></p>

>linear.x = (wheel_front_left + wheel_front_right + wheel_rear_left + wheel_rear_right) * (WHEEL_RADIUS/4)
>
>linear.y = ( -wheel_front_left + wheel_front_right + wheel_rear_left – wheel_rear_right) * (WHEEL_RADIUS/4)
>
>angular.z = ( -wheel_front_left + wheel_front_right – wheel_rear_left + wheel_rear_right) * (WHEEL_RADIUS/(4 * (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)))

<p>Source for mecanum wheel math: <a href="https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf">here.</a> </p>

## EX 1.
```c
void onTwist(const geometry_msgs::Twist &msg)
{
  if (connected)
  {
    // Cap values at [-1 .. 1]
    x = max(min(msg.linear.x, 1.0f), -1.0f);
    y = max(min(msg.linear.y, 1.0f), -1.0f);
    z = max(min(msg.angular.z, 1.0f), -1.0f);
    movement = true;
  }
  else
    stop();
}

void handleMovement()
{
  if (!movement || updating)
    return;

  // Mecanum drive:
  // ------------------------
  
  // Taken and simplified from: http://robotsforroboticists.com/drive-kinematics/
  float lf = x - y - z * 0.5;
  float rf = x + y + z * 0.5;
  float lb = x + y - z * 0.5;
  float rb = x - y + z * 0.5;

  // Map values to PWM intensities. 
  // PWMRANGE = full speed, PWM_MIN = the minimal amount of power at which the motors begin moving.
  uint16_t lfPwm = mapPwm(fabs(lf), PWM_MIN, PWMRANGE);
  uint16_t rfPwm = mapPwm(fabs(rf), PWM_MIN, PWMRANGE);
  uint16_t lbPwm = mapPwm(fabs(lb), PWM_MIN, PWMRANGE);
  uint16_t rbPwm = mapPwm(fabs(rb), PWM_MIN, PWMRANGE);

//  Serial.printf("%f, %f, %f, %f\n", lf, rf, lb, rb);
//  Serial.printf("---%d, %d, %d, %d\n", lfPwm, rfPwm, lbPwm, rbPwm);
//  Serial.printf("------%d, %d, %d, %d\n", lfPwm * (lf > 0), lfPwm * (lf < 0), lbPwm * (lb > 0), lbPwm * (lb < 0));

  // Each wheel has a channel for forward and backward movement
  pwm.writePWM(LF_FORW, lfPwm * (lf > 0));
  pwm.writePWM(LF_BACK, lfPwm * (lf < 0));
  pwm.writePWM(LB_FORW, lbPwm * (lb > 0));
  pwm.writePWM(LB_BACK, lbPwm * (lb < 0));
  
  pwm.writePWM(RF_FORW, rfPwm * (rf > 0));
  pwm.writePWM(RF_BACK, rfPwm * (rf < 0));
  pwm.writePWM(RB_FORW, rbPwm * (rb > 0));
  pwm.writePWM(RB_BACK, rbPwm * (rb < 0));

  movement = false;
}
```
## Ex 2.

<p>credit : https://ecam-eurobot.github.io/Tutorials/software/mecanum/mecanum.html</p>

#### $\color[rgb]{1,0,1}ROS$ $\color[rgb]{1,0,1}Twist$

<p>In the ROS navigations stack, all movements are indicated by Twist messages. These messages contain linear and angular velocity components and are often used to express the global movement of the base.</p>
<p>To control the 4 motors, we need to convert those velocities in angular velocities for each wheel. In our ROS node, we can define the following function:</p>

```c
def convert(move):
    x = move.linear.x
    y = move.linear.y
    rot = move.angular.z

    front_left = (x - y - rot * WHEEL_GEOMETRY) / WHEEL_RADIUS
    front_right = (x + y + rot * WHEEL_GEOMETRY) / WHEEL_RADIUS
    back_left = (x + y - rot * WHEEL_GEOMETRY) / WHEEL_RADIUS
    back_right = (x - y + rot * WHEEL_GEOMETRY) / WHEEL_RADIUS
```

<p>We used the inverse kinematic equations presented in the mecanum wheels chapter to convert global base velocity into individual angular velocities.</p>

#### $\color[rgb]{1,0,1}ROS$ $\color[rgb]{1,0,1}node$

<p>Now that we have a function to transform the twist message, let's setup the node to subscribe to Twist messages and publish individual angular velocity messages to specific topics:</p>

```c
rospy.init_node('mecanum')

# Get parameters about the geometry of the wheels
WHEEL_SEPARATION_WIDTH = rospy.get_param("/wheel/separation/horizontal")
WHEEL_SEPARATION_LENGTH = rospy.get_param("/wheel/separation/vertical")
WHEEL_GEOMETRY = (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) / 2
WHEEL_RADIUS = rospy.get_param("/wheel/diameter") / 2


pub_mfl = rospy.Publisher('motor/front/left', Float32, queue_size=1)
pub_mfr = rospy.Publisher('motor/front/right', Float32, queue_size=1)
pub_mbl = rospy.Publisher('motor/rear/left', Float32, queue_size=1)
pub_mbr = rospy.Publisher('motor/rear/right', Float32, queue_size=1)

sub = rospy.Subscriber('cmd_vel', Twist, convert)
rospy.spin()
```

<p>In this extract, we initialize the node, get some parameters from ROS' parameter server, define a publisher for each mecanum wheel and subscribe to the cmd_vel topic where Twist messages are send to. In the subscription, we provide our convertion function as a callback. This means that whenever a twist message is published on the topic, our function will be called with the twist message as argument.</p>
<p>The only thing left to do is to adapt our function to pusblish the correct values to the corresponding topics.</p>
<p>The code can be found in the Eurobot-2018 repository in the mecanum package</p>
