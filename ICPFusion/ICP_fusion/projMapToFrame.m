function [proj_map, proj_flag] = projMapToFrame(fusion_map, h, w, tform, cam_param)
    
    %==== Set parameters ====
    fx = cam_param(1);
    fy = cam_param(2);
    cx = cam_param(3);
    cy = cam_param(4);

    %==== TODO: Project all terms in the fusion map based on other input parameters ====
    %==== (Hint 1: Only project the points in front of the camera) ====
    %==== (Hint 2: Calculate all the projected indices together and use them in vectorization) ====
    %==== (Hint 3: Discard the indices that exceed the frame boundaries) ====

    % Write your code here...
    K=[fx,0,cx;0,fy,cy;0,0,1];
    proj_flag = true(size(fusion_map.pointcloud.Location, 1),1);
    fusion_map_camera_frame = pctransform(fusion_map.pointcloud,invert(tform));
    in_front_mask = (fusion_map_camera_frame.Location(:, 3) > 0);
    proj_flag = and(proj_flag,in_front_mask);
    in_front_mask = repmat(in_front_mask, 1, 3);
    valid_points = fusion_map_camera_frame.Location.*in_front_mask;
    image_coord_homog = (K*(valid_points'))';
    image_coord = image_coord_homog./image_coord_homog(1:end,3);
    %image_coord = image_coord(1:end,1:2);
    valid_points=reshape(image_coord, [], 1);
    % Point cloud stores in this format x,y,l = w,h,c
    boundary_mask = image_coord(1:end,1)>0 & image_coord(1:end,1)<=w & image_coord(1:end,2)>0 & image_coord(1:end,2)<=h;
    valid_points = ceil(valid_points);
    proj_flag = and(proj_flag,boundary_mask);
    
%     proj_points = zeros(h * w, 3);
%     proj_colors = zeros(h * w, 3);
%     proj_normals = zeros(h * w, 3);
%     proj_ccounts = zeros(h * w, 1);
%     proj_times = zeros(h* w, 1);
%     
%     pixel_points = reshape(fusion_map.pointcloud.Location(repmat(proj_flag, 1, 3)), [], 3);
%     pixel_colors = reshape(fusion_map.pointcloud.Color(repmat(proj_flag, 1, 3)), [], 3);
%     pixel_normals = reshape(fusion_map.normals(repmat(proj_flag, 1, 3)), [], 3);
%     pixel_ccounts = fusion_map.ccounts(proj_flag);
%     pixel_times = fusion_map.times(proj_flag);
%     pixel_locations = reshape(valid_points(repmat(proj_flag, 1, 3)), [], 3);
%     
    
%     a=fusion_map.pointcloud.Location.*repmat(proj_flag,1,3);
%     b=double(fusion_map.pointcloud.Color);
%     b=b.*repmat(proj_flag,1,3);
%     c=fusion_map.normals.*repmat(proj_flag,1,3);
%     d=fusion_map.ccounts.*proj_flag;
%     e=fusion_map.times.*proj_flag;
%     
%     proj_points = reshape(a, [h,w, 3]);
%     proj_colors=  reshape(b, [h,w, 3]);
%     proj_normals= reshape(c, [h,w, 3]);
%     proj_ccounts= reshape(d, [h,w,1]);
%     proj_times=   reshape(e,[h,w,1]);
    
    
%     proj_points = reshape(fusion_map.pointcloud.Location.*repmat(proj_flag,1,3), [h,w, 3]);
%     proj_colors=  reshape(double(fusion_map.pointcloud.Color).*repmat(proj_flag,1,3), [h,w, 3]);
%     proj_normals= reshape(fusion_map.normals.*repmat(proj_flag,1,3), [h,w, 3]);
%     proj_ccounts= reshape(fusion_map.ccounts.*proj_flag, [h,w,1]);
%     proj_times=   reshape(fusion_map.times.*proj_flag,[h,w,1]);
%     
%     
%     x=5;
%     
    
    proj_points = zeros(h * w, 3);
    proj_colors = zeros(h * w, 3);
    proj_normals = zeros(h * w, 3);
    proj_ccounts = zeros(h * w, 1);
    proj_times = zeros(h* w, 1);
    
    pixel_points = reshape(fusion_map.pointcloud.Location(repmat(proj_flag, 1, 3)), [], 3);
    pixel_colors = reshape(fusion_map.pointcloud.Color(repmat(proj_flag, 1, 3)), [], 3);
    pixel_normals = reshape(fusion_map.normals(repmat(proj_flag, 1, 3)), [], 3);
    pixel_ccounts = fusion_map.ccounts(proj_flag);
    pixel_times = fusion_map.times(proj_flag);
    pixel_locations = reshape(valid_points(repmat(proj_flag, 1, 3)), [], 3);
    
    proj_points((pixel_locations(:, 1)-1) * h + pixel_locations(:, 2), :) = pixel_points;
    proj_colors((pixel_locations(:, 1)-1) * h + pixel_locations(:, 2), :) = pixel_colors;
    proj_normals((pixel_locations(:, 1)-1) * h + pixel_locations(:, 2), :) = pixel_normals;
    proj_ccounts((pixel_locations(:, 1)-1) * h + pixel_locations(:, 2)) = pixel_ccounts;
    proj_times((pixel_locations(:, 1)-1) * h + pixel_locations(:, 2)) = pixel_times;
    
    proj_points = reshape(proj_points, [h, w, 3]);
    proj_colors = reshape(proj_colors, [h, w, 3]);
    proj_normals = reshape(proj_normals, [h, w, 3]);
    proj_ccounts = reshape(proj_ccounts, [h, w, 1]);
    proj_times = reshape(proj_times, [h, w, 1]);
    
    
    
    
    


    %==== Output the projected map in a struct ====
    %==== (Notice: proj_points[], proj_colors[], and proj_normals[] are all 3D matrices with size h*w*3) ====
    %==== (Notice: proj_ccounts[] and proj_times[] are both 2D matrices with size h*w) ====
    proj_map = struct('points', proj_points, 'colors', proj_colors, 'normals', proj_normals, 'ccounts', proj_ccounts, 'times', proj_times);
        
end
