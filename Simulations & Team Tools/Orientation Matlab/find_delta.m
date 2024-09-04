function delta = find_delta(phi, gamma)
%This function finds the difference between phi and gamma

if phi < 0
    phi = phi + 360;
end

delta = gamma - phi;

end