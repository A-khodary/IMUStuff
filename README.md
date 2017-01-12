# IMU stuff

## Why does the pitch only go between -90 degrees and +90 degrees?

This has been a common question so I thought I’d say something about why pitch behaves differently to roll and yaw, at least in RTIMULib(2). I qualify this because other software may have different conventions and it may not always be the pitch axis that is different to the other two.

The convention RTIMULib(2) uses is that roll and yaw vary between -180 degrees and +180 degrees but pitch varies between -90 degrees and +90 degrees. Why doesn’t pitch also vary between -180 degrees and +180 degrees? It’s because a sphere is fully represented by these ranges and so it is unnecessary for all three axes to have the full range.

Take the case where the IMU is at zero roll and yaw and pitch is steadily increasing from zero. When it goes just past straight up, the roll and yaw will change by 180 degrees to reflect that it has gone past the vertical. This discontinuity is unpleasant but does represent the pose correctly. These discontinuities are why it is such a good idea to use quaternions whenever possible and only resort to Euler angles when absolutely necessary.

As a concrete example, let’s say that an IMU, parallel to the ground and facing north is taken 45 degrees past the vertical to a pitch that might be considered 135 degrees. This could be described as having a roll of zero, a pitch of 135 degrees and a yaw of zero. Alternatively, this could be regarded as having a roll of 180 degrees, a pitch of 45 degrees and a yaw of 180 degrees. These two poses are identical and is a form of aliasing. Likewise, there are two ways of forming a quaternion for each pose. It’s always possible to convert from one to another and this could be done if an application required it.

For more about ranges of values, check out the Wikipedia entry for Euler angles here.  I probably should also mention how difficult it is to get the pitch to achieve exactly -90 degrees or +90 degrees. This is largely because it is a singular point (well two singular points really) and on either side of the point the pitch will be less. It’s instructive to look at how the pitch angle is derived from the underlying quaternion:

    vec.setY(asin(2.0 * (m_data[0] * m_data[2] - m_data[1] * m_data[3])));
  
m_data is the normalized quaternion representing the IMU pose. In order for the asin() result to be +90 degrees for example, this part:

    m_data[0] * m_data[2] - m_data[1] * m_data[3]

has to exactly equal 0.5 and this will only happen if everything is exactly right. Due to issues with calibration errors, non-orthogonal axes, non-aligned axes between sensors and things like that which are tough to completely eradicate when using a MEMS IMU, it may be that it is impossible for the filter to reliably output a pose representing a pitch of exactly +/90 degrees.

For those interested, it is a worthwhile experiment to actually try this. It requires moving the IMU around by very small amounts using trial and error to try to maximize the pitch. If you are lucky, you might actually see +/-90 degrees!

## Using quaternions to correct IMU installation offsets

RTIMULib supports the correction of arbitrary, non-standard, coordinate axis-aligned IMU orientations. However, it is very often true that an installation of an IMU in a device isn’t completely perfect, leading to offsets of a few degrees in each axis perhaps. It’s very easy to correct this using quaternions however.

In order to do this, the offset orientation must be captured by putting the device in its reference position and storing the fused quaternion – fusionQPose. Let’s call this referenceQPose. Then, every time a fused result from the IMU is used, simply pre-multiply the fusionQPose by the conjugate of the referenceQPose:

    correctedQPose = referenceQPose.conjugate() * fusionQPose;
    correctedQPose.toEuler(correctedPose);
  
It’s as easy as that.

## Using multiple IMUs with RTIMULib

A typical initialization section of an RTIMULib app looks like this:

    RTIMUSettings *settings = new RTIMUSettings("RTIMULib");
    RTIMU *imu = RTIMU::createIMU(settings);
    if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)) {
         printf("No IMU found");
         exit(1);
    }
    imu->IMUInit();
    
To use more than one IMU, just do something like this instead:

    // create settings0 and imu0

        RTIMUSettings *settings0 = new RTIMUSettings("RTIMULib0");

    // IMU type, bus and address and other interface parameters
    // along with any associated pressure sensor address and type 
    // can be overridden in the settings object here.

        RTIMU *imu0 = RTIMU::createIMU(settings0);
        if ((imu0 == NULL) || (imu0->IMUType() == RTIMU_TYPE_NULL)) {
            printf("No IMU found");
            exit(1);
        }
    // This is an opportunity to manually override any other settings
    // set up IMU

        imu0->IMUInit();

    // create settings1 and imu1

        RTIMUSettings *settings1 = new RTIMUSettings("RTIMULib1");

    // IMU type, bus and address and other interface parameters
    // along with any associated pressure sensor address and type 
    // can be overridden in the settings object here.

        RTIMU *imu1 = RTIMU::createIMU(settings1);
        if ((imu1 == NULL) || (imu1->;IMUType() == RTIMU_TYPE_NULL)) { 
            printf("No IMU found"); 
         exit(1); 
        }

    // This is an opportunity to manually override any other settings
    // set up IMU

        imu1->;IMUInit();
    
Once the initialization is done, just poll imu0 and imu1 as normal. The code above will create an RTIMULib0.ini file and an RTIMULib1.ini file. If you just run the program and these don’t exist, RTIMULib will create each file in turn and try to auto-detect an IMU. Right now, both will be set to the same IMU (the first one found) so one of the files would have to be hand edited to use the second IMU. An alternative to this is to put code just before the call to createIMU that modifies the appropriate variables in the settings1 object. This will prevent auto-detect running and use the supplied type, bus and address information.

If either or both of the IMUs have pressure sensors, then the auto-detect will associate the first pressure sensor found with the first IMU found. If this is not the desired behavior then this too must be overridden.

Note that there is no need for the two IMUs to be the same chips or on the same bus – they can be on different I2C busses, different SPI busses or I2C and SPI. And there is no limit to the total number of IMUs that can be supported (although things like bus utilization and CPU loading must be taken into account).

In order to calibrate the magnetometers and accelerometers, the RTIMUCal app can be used as normal but with the specific names as parameters:

    RTIMULibCal RTIMULib0
    RTIMULibCal RTIMULib1

This will set the calibration data in the correct .ini files.

## This is why trying to measure position with a MEMS 9-dof IMU is doomed to failure

Most of us have been there. We have a 9-dof MEMS IMU nicely measuring pose (i.e. orientation in space) and then it seems like a good idea to try to use the same hardware to track position changes. This post talks about why we have all (as far as I am aware) failed.

First off, let’s review how data fusion works with a 9-dof IMU to obtain orientation in space. A 9-dof IMU has three sensors, each with different roles:

* Gyros. These are really the primary sensors in many ways. They provide virtually instantaneous response to orientation changes with relatively low noise. Since they output angular rates, the outputs have to be (discrete) integrated with respect to time to obtain angle changes. However, gyros have no sense of up or down – they contain no absolute reference. They merely sense orientation changes.

* Accelerometers. When using an IMU for orientation sensing, the accelerometers’ role is primarily to track the direction of the gravity vector, since this is known to be straight down. Of course, accelerometers aren’t just sensitive to gravitational acceleration – they sense absolutely every small disturbance and can be inherently noisy. However, when nicely filtered, the outputs can be used to maintain an absolute reference to the horizontal plane – i.e. they provide a reference for pitch and roll. They give no information about yaw (heading). One problem they have is when the IMU is subjected to continuous acceleration (on a moving vehicle under acceleration for example or an aircraft in a turn), the reference will be distorted. It is possible to externally measure the speed of the vehicle and this can be used to correct for these accelerations but it is a potential source of error.

* Magnetometers. Magnetometers provide the missing yaw reference by sensing the Earth’s magnetic field. However, what they are really doing is sensing the ambient magnetic field which is affected by almost anything – nearby magnets, magnetic materials, which way the wind is blowing etc.

The task of 9-dof fusion software is to take these three sets of measurements (and instantaneous vehicle speed if corrections are necessary) and come up with a (mostly) correct orientation.

Ok, so that’s orientation. Now, position. To try and sense positional changes using an IMU, you basically have to do two things:

1. Remove the gravity vector to leave the residual accelerations.
2. Double integrate the residual accelerations with respect to time to give a change in position.

Simple enough in principle – what could go wrong? Well basically everything. For one thing, this piecewise linear integration which basically assumes a constant acceleration for a time step (sample interval) is probably not going to be exactly correct. Given that the accelerations needs to be double integrated, this can cause significant error. Any error in subtracting the rotated gravity vector to obtain residual accelerations will similarly cause large errors in position. Noise in the accelerometers or incorrect calibration will also lead to errors. Etc etc.

So there are many sources of errors. The problem is that there is no way to correct them. Suppose for example that the IMU is actually at rest but, due to previous errors, a small velocity is left. Even with zero residual accelerations, the code will think that the IMU is still moving and keep changing position accordingly. Basically, the whole thing diverges pretty quickly!

Is there a fix? Well using super-duper military class sensors (laser ring gyros and the like) with super accurate mounts and excellent engineering all round will work a lot better. Other approaches use multiple IMUs and combine the outputs to try to counteract the various errors. This does potentially lead to better performance but is probably still not good enough. For example, MEMS gyros are affected by linear accelerations, not just rotations. So they need to be isolated from vibrations to counteract that.

As far as I am aware, the only realistic way to use MEMS IMUs for position sensing is to have a separate absolute reference that can be used to correct errors and stop the divergence. This is basically the same idea as using accelerometers and magnetometers to correct errors in the gyro angle rate data. The problem is that, if such a reference exists, why not just use that for position sensing instead?

## Connecting multiple identical IMUs (or other I2C devices) to a single I2C interface

There are many situations where multiple I2C devices need to be connected to a single processor. Smart gloves and motion capture suits are a couple of examples. The issue is that, typically for IMUs anyway, there are only two I2C address options. Using an SPI interface results in less of a problem as a separate select line is required for each device anyway. Even if enough GPIOs aren’t available, a simple demux chip will expand a few GPIOs to a larger number. Using multiple I2C interfaces obviously helps but most most processors only expose one or two.

If I2C is the only option available, there are a few ways to expand the number of devices that can be connected to a single I2C interface that work for any chip. What’s needed is a technique that hides devices that aren’t being addressed and allows the processor to talk only to the desired chip.

One thing to always remember is that you can’t ignore signal integrity. Multiple identical sensor chips usually means that they are going to located in distinct places so lengths of cable are involved. I2C can be successfully run over a few feet of cable (I typically use the USB cable for this kind of thing) provided attention is paid to optimizing the pull-up resistor values.

So, how to fix the limited addressing problem?

* Use an I2C switch. An example is the TI TCA9548A. This is itself an I2C device and allows up to eight I2C devices (there are two and four port options as well) to connected to it, along with the master I2C interface. Essentially, the processor tells the switch which interface to activate via the switch chip’s own I2C address and then the processor can address the selected sensor chip as though it were directly connected. All eight devices can be at the same I2C address. A nice aspect of this is that each cable run to the sensor is isolated from any others. You can connect up to eight of these switches for a total of 64 sensors! Not sure anyone would really want to do that.
* Use the two available I2C addresses as an enable. This is a bit like having a separate chip select for each sensor. Using a set of GPIOs (or GPIOs with demux to get more select lines), connect one line to the address select input of each of the sensor chips. In normal operation all lines are at a high level except for one. So, all of the sensors will be at one address with just a single sensor at the other. The processor can then talk to this sensor alone without affecting the others. This system probably works (I haven’t tried it myself) but has a significant disadvantage. There is no electrical isolation between the different sensors. That will make signal integrity a real problem due to all the capacitance added by the cables. Plus termination becomes very tricky if not impossible. Plus you need five wires to every sensor.
* Use an analogue switch. This is a bit like the first option except that (in principle) any old analogue switch can be used. This means that the SDA and SCL signals from the processor only go to one device and each cable run is isolated. I haven’t tried this – a purpose-built I2C switch seems like a better solution.

Having said all of this, I think that connecting too many sensors to one I2C interface is just a bad idea – especially for high sample rate IMU applications. For one thing, the total bandwidth of the interface is split between all the devices on the same interface. Even at 400kHz, this could be a significant limitation. Plus I have mentioned the signal integrity issues a few times now. Strange hang-ups and odd data could well be the result of too much cable and incorrect termination.

What to do then? Depends on the application but I think serious thought should be given to having only one or two sensors connected to a low cost, battery powered, wireless connected processor and then replicating this to get the desired total number of sensors. The processor itself doesn’t have to do anything very complicated – just operate the chips and send the raw data back to a more powerful processor.

## Magnetometer tilt compensation using quaternions

There are many techniques out there for tilt compensating a magnetometer but I use a quaternion-based technique that seems to work very nicely. It has always been used in all flavors of RTIMULib. Tilt compensation is necessary because the magnetometer is reading the magnetic field in the orientation of the object in which the magnetometer is mounted whereas the required measurements are in the horizontal plane. Effectively, the roll and pitch angles need to be undone to make it look as though the magnetometer is horizontal. Things always get complicated at high roll and pitch angles but this is where quaternions come in handy. Code snippets after the jump for how this works…

The first thing needed is the roll and pitch information. The convention used here is that, if the device is an aircraft for example, then the x-axis points forward from the nose as seen by the pilot, the y-axis points out along the right-wing and the z-axis points straight down. So, a positive roll angle means that the right-wing rolls down, a positive pitch angle means that the nose points up and a positive yaw angle means that the nose rotates towards the right.

This is a version of the code in RTIMULIb:

    RTQuaternion q;    // this is a quaternion derived from roll and pitch
    RTQuaternion m;    // this is the magnetometer data as a quaternion

    // the first part constructs quaternion from roll and pitch
 
    RTFLOAT cosX2 = cos(roll / 2.0f);
    RTFLOAT sinX2 = sin(roll / 2.0f);
    RTFLOAT cosY2 = cos(pitch / 2.0f);
    RTFLOAT sinY2 = sin(pitch / 2.0f);

    q.setScalar(cosX2 * cosY2);
    q.setX(sinX2 * cosY2);
    q.setY(cosX2 * sinY2);
    q.setZ(-sinX2 * sinY2);

    // now construct a quaternion from the magnetometer data
    m.setScalar(0);
    m.setX(mag.x());
    m.setY(mag.y());
    m.setZ(mag.z());

    // perform the tilt compensation
    m = q * m * q.conjugate();

    // calculate the yaw value
    yaw = -atan2(m.y(), m.x()));
    
The input to this code snippet is the pitch and roll in radians. This is derived from the accelerometers. The first part of the code creates a quaternion q that represents the roll and pitch – all that stuff with sin and cos is how you create a quaternion from Euler angles (yaw is always 0 for this purpose so this code has been optimized to reflect that).

Then, the magnetometer vector is put into a quaternion, ready to be tilt compensated. The magic happens in the line before last. This is where the magnetometer information is rotated by the quaternion from the roll and pitch and becomes the tilt compensated version (i.e. the values you would get if the magnetometer was perfectly horizontal). The last line is the standard way of generating the yaw value from the tilt compensated magnetometer readings.

