[https://stackoverflow.com/questions/42809088/how-to-validate-a-xml-file-with-xsd-through-xmllint](https://stackoverflow.com/questions/42809088/how-to-validate-a-xml-file-with-xsd-through-xmllint)

~~~bash
xmllint --schema ~/catkin_ws/build/adore_if_ros/_deps/xodr-src/schema/OpenDRIVE_1.4H.xsd yourfile.xodr --noout
~~~