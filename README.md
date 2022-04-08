# Simulation-based Robot Verification Testing Tool (SRVT)
[![CodeFactor](https://www.codefactor.io/repository/github/akerdogmus/srvt-ros/badge)](https://www.codefactor.io/repository/github/akerdogmus/srvt-ros) ![current_version](https://img.shields.io/github/v/release/inomuh/srvt-ros?color=green) ![last_commit](https://img.shields.io/github/last-commit/inomuh/srvt-ros?color=green) ![repo_size](https://img.shields.io/github/repo-size/inomuh/srvt-ros) ![Apache-2.0 License](https://img.shields.io/github/license/inomuh/srvt-ros?color=blue) ![lang](https://img.shields.io/github/languages/top/inomuh/srvt-ros)

SRVT can be thought of as a toolkit or advanced method that allows a robotic system to be imported into a simulation environment and applied to validation tests. The basis of the system is the coordinated use of some critical software for the ROS ecosystem. Simulation environment using Gazebo, trajectory planning using Moveit, mission communication and dynamic verification system using ROS Smach package were built in a single ROS package. For more information about the purpose of this tool: https://dergipark.org.tr/en/pub/jster/issue/61588/979689

#### Note: This tool does not include the robot model files, you can access the model addition details from the "Tool Environment Setup" section below.

### Tool Environment Setup

- For English documentation: [Eng](https://github.com/inomuh/srvt-ros/blob/main/docs/tutorial_en.md)
- For Turkish documentation: [Tur](https://github.com/inomuh/srvt-ros/blob/main/docs/tutorial_tr.md)

Changelog:
----------
v1.0 - 17.02.22
----------------------
- First Commit

v1.0.1 - 17.02.22
----------------------
- Bugfixes

v2.0 - 08.04.22
----------------------
- All codes and subpackages revised

v2.1 - 08.04.22
---------------------
- v2.0's bugfixes

v2.1.1 - 08.04.22
---------------------
- Minor changes on srvt_moveit launch files.

### Credits

<a href="http://valu3s.eu">
  <img align=left img src="https://upload.wikimedia.org/wikipedia/tr/d/d0/TUBITAK-Logo.jpg" 
       alt="tübitak_logo" height="100" >
</a>

---

This work is supported by [TÜBİTAK](https://www.tubitak.gov.tr/) Project under grant number 120N803 which conducted by the İnovasyon Mühendislik.

---

<a href="http://valu3s.eu">
  <img align=right img src="https://valu3s.eu/wp-content/uploads/2020/04/VALU3S_green_transparent-1024x576.png" 
       alt="valu3s_logo" height="100" >
</a>

  This work is also done by [Inovasyon Muhendislik](https://www.inovasyonmuhendislik.com/) and [ESOGU-SRLAB](https://srlab.ogu.edu.tr/) under [VALU3S](https://valu3s.eu) project. This project has received funding from the [ECSEL](https://www.ecsel.eu) Joint Undertaking (JU) under grant agreement No 876852. The JU receives support from the European Union’s Horizon 2020 research and innovation programme and Austria, Czech Republic, Germany, Ireland, Italy, Portugal, Spain, Sweden, Turkey.

## Cite

If the code or data help you, please cite the following the paper.

 	@research article{jster979689, 
    journal = {Journal of Science, Technology and Engineering Research},
    issn = {}, 
    eissn = {2717-8404},
    address = {Eymir mah. Tek küme evleri, No.59/6 Gölbaşı-ANKARA},
    publisher = {Mehmet BULUT},
    year = {2021},
    volume = {2},
    pages = {31 - 45},
    doi = {10.53525/jster.979689},
    title = {Endüstriyel Robot Hareket Planlama Algoritmaları Performans Karşılaştırması},
    key = {cite},
    author = {Erdoğmuş, Alim} }

### License

See the [LICENSE](LICENSE.md) file for license rights and limitations (Apache-2.0 Licence).
  
