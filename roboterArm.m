function robot=roboterArm(sizes,maxDist)
if sum(sizes)<=maxDist
    error('arm to small')
end
robot = rigidBodyTree('DataFormat','column','MaxNumBodies',3);

body = rigidBody("link1");
joint = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'base');

for i = 1:length(sizes)-1
    l="link"+(i+1)
    j="joint"+(i+1)
    l2="link"+i
    body = rigidBody(l);
    joint = rigidBodyJoint(j, 'revolute');
    setFixedTransform(joint,trvec2tform([sizes(i) 0 0]));
    joint.JointAxis = [0 0 1];
    body.Joint = joint;
    addBody(robot, body, l2);
end

body = rigidBody('endE_tool');
joint = rigidBodyJoint('fix1','fixed');
setFixedTransform(joint, trvec2tform([sizes(end), 0, 0]));
body.Joint = joint;
addBody(robot, body, "link"+(length(sizes)));

showdetails(robot)