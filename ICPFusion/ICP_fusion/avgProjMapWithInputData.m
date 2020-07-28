function updated_map = avgProjMapWithInputData(proj_map, input_data, alpha, h, w, is_use, t)

    %==== Set variables ====
    input_points = input_data.pointcloud.Location;
    input_colors = input_data.pointcloud.Color;
    input_normals = input_data.normals;
    proj_points = proj_map.points;
    proj_colors = proj_map.colors;
    proj_normals = proj_map.normals;
    proj_ccounts = proj_map.ccounts;
    proj_times = proj_map.times;

    %==== TODO: Update all the terms in the projected map using the input data ====
    %==== (Hint: apply is_use[] as a mask in vectorization) ====

    % Write your code here...
    
    alpha1= alpha.*is_use;
    input_points1 = input_points;
    input_colors1 = double(input_colors);
    input_normals1= input_normals;
    proj_times1 = proj_times;
    
    updated_points = (proj_ccounts.*proj_points + alpha1.*input_points1)./(proj_ccounts+alpha1);
    updated_normals = (proj_ccounts.*proj_normals + alpha1.*input_normals1)./(proj_ccounts+alpha1);
    updated_colors = (proj_ccounts.*proj_colors + alpha1.*input_colors1)./(proj_ccounts+alpha1);
    updated_ccounts = proj_ccounts+ alpha1;
    updated_times= proj_times1 + t;


    
    %==== Output the updated projected map in a struct ====
    updated_map = struct('points', updated_points, 'colors', updated_colors, 'normals', updated_normals, 'ccounts', updated_ccounts, 'times', updated_times);
        
end