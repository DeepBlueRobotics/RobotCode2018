Place all third-party libraries (*.jar and *.so) directly in this directory,
not a subdirectory.

The `userLibs.dir` property in `../build.properties` is set to point to
this directory so that the FRC plugin will look here for third-party libs
(*.jar, and *.so) instead of in `${user.home}/wpilib/user/java/lib`. This
ensures that those libs can be versioned with the code and don't need to
be manually installed on each developer's machine. The FRC plugin will
automatically add all jars in this directory to the classpath and will
deploy the jars and .so files to the roboRIO along with our code.