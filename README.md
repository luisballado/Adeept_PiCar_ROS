# Adeept_PiCar_ROS
ROS based control for Adeept Pi Car with line following proof of concept

Make sure to append and source the .bashrc variables before you begin!

# Dependencies:

  Both:

    ROS (Melodic in this case)
    
        https://www.ros.org/
    
        http://wiki.ros.org/melodic/Installation
    
    Gstreamer
    
        sudo apt-get install gstreamer1.0
  
  Car:
    
    Adafruit_PCA9685
    
        https://github.com/adafruit/Adafruit_Python_PCA9685
    
  Laptop:
  
    OpenCV (Compiled with Gstreamer support)
    
        https://medium.com/@galaktyk01/how-to-build-opencv-with-gstreamer-b11668fa09c

# Startup:

  Car:
  
    roscore
  
    ROS.py
  
    GST_Vid.sh
  
  Laptop:
  
    ROS_ini.py
  
    Either: 
    
      - Picar_vid.py 
      
      - Picar_auto.py
  
# Have fun!
  
