# xtrc_llc
low level control ROS2 package for xtoyz_rc_car listening on cmd_vel topic using Twist msg interface
Car available here: https://www.amazon.com/dp/B08F9SRHKQ?psc=1&ref=ppx_yo2_dt_b_product_details

pkg intended to control motors of car with L298N driver board: https://www.amazon.com/HiLetgo-Controller-Stepper-H-Bridge-Mega2560/dp/B07BK1QL5T/ref=sr_1_2?crid=8A6GCDA6LIQQ&keywords=l298n+motor+driver+module&qid=1639005466&sprefix=l298n+moto%2Ctoys-and-games%2C165&sr=8-2

also using a 7.2 volt battery as input to driver board, however these motors aren't rated for more than 4.5 volt so limiting duty cycle accordingly to not overload motors
