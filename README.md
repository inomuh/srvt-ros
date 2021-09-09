
## SRVT ROKOS ROS Paketleri Kurulum&Kullanım Rehberi ##

### SRVT'nin çalıştırılabilmesi için yapılması gereken kurulumlar:
------------------------------------------------------------------

- ROS Noetic kurulumu için: 

```bash
http://wiki.ros.org/noetic/Installation/Ubuntu
```

- ROS bağımlılıklarının kurulumları için aşağıdaki komut çalıştırılmalıdır:

```bash
sudo apt-get install ros-noetic-moveit && sudo apt-get install ros-noetic-controller-manager && sudo apt-get install ros-noetic-joint-trajectory-controller && sudo apt-get install ros-noetic-rqt-joint-trajectory-controller && sudo apt-get install ros-noetic-effort-controllers
```
- Daha sonra rosdep komutları çalıştırılmalıdır.

```bash
sudo rosdep init && rosdep update
```
- Son olarak "otokar_simulation" ROS paketi, ROS çalışma alanına atılmalı ve derlenmelidir.

```bash
cd ~/catkin_ws && catkin_make && catkin_make install
```

### Model Dosyalarının Düzenlenmesi

- "model" dosyasındaki modelleri, ".gazebo/models" klasörünün içerisine ve "otokar_simulation" klasörüne çıkartın.

  ##### NOT: Eğer daha önce gazebo çalıştırılmadıysa .gazebo dosyasının oluşması için terminalden "gazebo" komutu yazılarak gazebo çalıştırılmalıdır.
  ##### NOT2: "models" klasörü yoksa oluşturulmalıdır.

- "otokar_description_v2" dosyasını SRVT-git reposundan indirilen "otokar_simulation" içerisine atın.

- modellerin çalışması için ~/.bashrc dosya içeriği düzenlemelerini yapın.

### ~/.bashrc Dosyası İçeriği Düzenlemeler ###

- Aşağıdaki komutların ~/.bashrc dosyasına eklenmesi gerekir.

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source /usr/share/gazebo-11/setup.sh
export GAZEBO_MODEL_PATH=~/catkin_ws/src/otokar_simulation/model/:$GAZEBO_MODEL_PATH
```

### SRVT'nin Kullanım Kodları
 
- ROKOS'un çalıştırılması için sırasıyla ve ayrı terminallerde aşağıdaki komutlar çalıştırılmalıdır.

ROKOS'u çalıştırmak için:

```bash
roslaunch rokos_moveit start_system.launch
```

Image Server düğümünü çalıştırmak için:

```bash
rosrun rokos_moveit image_service_node.py
```

Moveit planlayıcıyı çalıştırmak için:

```bash
roslaunch rokos_moveit start_moveit.launch
```

Task Service'i çalıştırmak için:

```bash
roslaunch rokos_moveit start_rokos_task_service.launch
```

Smach düğümünü çalıştırmak için:

```bash	
roslaunch rokos_moveit start_rokos_smach.launch
```
