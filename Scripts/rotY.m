%% Transformada de rotación de 4x4 en Y

function RY=rotY(theta)

s=sin(theta); c=cos(theta);
if isnumeric(theta)
    if abs(c)<1e-10
        c=0;
    end
    if abs(s)<1e-10
         s=0;
    end
end  

RY=[c,0,s,0;0,1,0,0;-s,0,c,0;0,0,0,1];