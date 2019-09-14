load full_system.mat
load controller.mat

[C_esqnum, C_esqden] = tfdata(C_esq);
C_esqnum = cell2mat(C_esqnum);
C_esqden = cell2mat(C_esqden);

[G_esqnum, G_esqden] = tfdata(G_esq);
G_esqnum = cell2mat(G_esqnum);
G_esqden = cell2mat(G_esqden);

[C_dirnum, C_dirden] = tfdata(C_dir);
C_dirnum = cell2mat(C_dirnum);
C_dirden = cell2mat(C_dirden);

[G_dirnum, G_dirden] = tfdata(G_dir);
G_dirnum = cell2mat(G_dirnum);
G_dirden = cell2mat(G_dirden);

% Equations:
% Left:
%   u[k] = 0.4806e[k-2] + 0.4u[k-1] + 0.6u[k-2]
%   u[negativo] = 0
%   e[negativo] = 0
%
% Right:
%   u[k] = 0.4373e[k-2] + 0.5075u[k-1] + 0.4925u[k-2]
%   u[negativo] = 0
%   e[negativo] = 0