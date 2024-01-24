pc1 = pcread('ex_lid.pcd');
pc2 = pcread('ex_zed.pcd');
 
%known transform
R = [ cos(-pi/2) sin(-pi/2) 0 0
     -sin(-pi/2) cos(-pi/2) 0 0
      0         0   1 0
      0         0   0.25 1];
tform = rigid3d(R);
pc2 = pctransform(pc2,tform);

pc2 = pcdownsample(pc2,"nonuniformGridSample",64);

%trim
roi = [-3 1 -7 -0.4 -0.6 2.5];
pc1 = select(pc1, findPointsInROI(pc1,roi));
pc2 = select(pc2, findPointsInROI(pc2,roi));

[~,pc2Reg, rmse] = pcregistericp(pc2, pc1, 'Metric', "pointToPlane", 'MaxIterations', 100);

% pcshowpair(pc1,pc2);
pcshowpair(pc1,pc2Reg, 'MarkerSize', 50);

disp(rmse)