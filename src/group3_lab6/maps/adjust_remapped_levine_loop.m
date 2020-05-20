close all;
clear all;
map = imread('remapped_levine_loop.pgm');
%imwrite(map, 'remapped_levine_loop.png', 'PNG');
binary_map = imbinarize(map,'global');
walls = bitxor(binary_map, imdilate(binary_map,strel('disk',3)));
map = zeros(size(map),'uint8');
map(find(binary_map))  = intmax('uint8');
map(find(~binary_map)) = intmax('uint8') * 0.8;
map(find(walls))       = intmin('uint8');
imwrite(map, 'adjusted_remapped_levine_loop.pgm', 'PGM');
%imwrite(map, 'adjusted_remapped_levine_loop.png', 'PNG');
imshow(map)

