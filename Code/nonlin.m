function f = nonlin(x)
vA_init =  evalin('base','vA_init');
phi_init =  evalin('base','phi_init');
psi_init =  evalin('base','psi_init');
h_init =  evalin('base','h_init');
f(1) = (x(1)*sin(x(3))*((981*cos(x(8))*sin(x(7)))/100 - x(6)*(-(x(1)^2*(sin(x(3))^2 - 1))/(tan(x(2))^2 + 1))^(1/2) + x(4)*tan(x(2))*(-(x(1)^2*(sin(x(3))^2 - 1))/(tan(x(2))^2 + 1))^(1/2) + (17563*x(1)^2*cos(x(3))*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((8*x(3))/5 - (6*x(14))/25))/(160*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160)) + (17563*x(1)^2*sin(x(3))*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((7*((11*x(2))/2 + 327/500)^2)/100 + 13/100))/(160*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160))) - (-(x(1)^2*(sin(x(3))^2 - 1))/(tan(x(2))^2 + 1))^(1/2)*((981*sin(x(8)))/100 - (981*x(12))/100 - x(6)*x(1)*sin(x(3)) + x(5)*tan(x(2))*(-(x(1)^2*(sin(x(3))^2 - 1))/(tan(x(2))^2 + 1))^(1/2) + (17563*x(1)^2*sin(x(2))*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((3947*x(2))/650 + (248*x(11))/325 + (79373*pi)/234000 + (15376*x(5))/(625*x(1))))/(160*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160)) + (17563*x(1)^2*cos(x(2))*sin(x(3))*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((8*x(3))/5 - (6*x(14))/25))/(160*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160)) - (17563*x(1)^2*cos(x(2))*cos(x(3))*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((7*((11*x(2))/2 + 327/500)^2)/100 + 13/100))/(160*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160))) + tan(x(2))*(-(x(1)^2*(sin(x(3))^2 - 1))/(tan(x(2))^2 + 1))^(1/2)*((981*cos(x(7))*cos(x(8)))/100 + x(5)*(-(x(1)^2*(sin(x(3))^2 - 1))/(tan(x(2))^2 + 1))^(1/2) - x(4)*x(1)*sin(x(3)) + (17563*x(1)^2*cos(x(2))*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((3947*x(2))/650 + (248*x(11))/325 + (79373*pi)/234000 + (15376*x(5))/(625*x(1))))/(160*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160)) - (17563*x(1)^2*sin(x(2))*sin(x(3))*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((8*x(3))/5 - (6*x(14))/25))/(160*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160)) + (17563*x(1)^2*cos(x(3))*sin(x(2))*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((7*((11*x(2))/2 + 327/500)^2)/100 + 13/100))/(160*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160))))/(x(1)^2*sin(x(3))^2 - (x(1)^2*(sin(x(3))^2 - 1))/(tan(x(2))^2 + 1) - (x(1)^2*tan(x(2))^2*(sin(x(3))^2 - 1))/(tan(x(2))^2 + 1))^(1/2);
f(2) = -((-(x(1)^2*(sin(x(3))^2 - 1))/(tan(x(2))^2 + 1))^(1/2)*((981*cos(x(7))*cos(x(8)))/100 + x(5)*(-(x(1)^2*(sin(x(3))^2 - 1))/(tan(x(2))^2 + 1))^(1/2) - x(4)*x(1)*sin(x(3)) + (17563*x(1)^2*cos(x(2))*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((3947*x(2))/650 + (248*x(11))/325 + (79373*pi)/234000 + (15376*x(5))/(625*x(1))))/(160*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160)) - (17563*x(1)^2*sin(x(2))*sin(x(3))*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((8*x(3))/5 - (6*x(14))/25))/(160*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160)) + (17563*x(1)^2*cos(x(3))*sin(x(2))*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((7*((11*x(2))/2 + 327/500)^2)/100 + 13/100))/(160*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160))) + tan(x(2))*(-(x(1)^2*(sin(x(3))^2 - 1))/(tan(x(2))^2 + 1))^(1/2)*((981*sin(x(8)))/100 - (981*x(12))/100 - x(6)*x(1)*sin(x(3)) + x(5)*tan(x(2))*(-(x(1)^2*(sin(x(3))^2 - 1))/(tan(x(2))^2 + 1))^(1/2) + (17563*x(1)^2*sin(x(2))*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((3947*x(2))/650 + (248*x(11))/325 + (79373*pi)/234000 + (15376*x(5))/(625*x(1))))/(160*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160)) + (17563*x(1)^2*cos(x(2))*sin(x(3))*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((8*x(3))/5 - (6*x(14))/25))/(160*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160)) - (17563*x(1)^2*cos(x(2))*cos(x(3))*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((7*((11*x(2))/2 + 327/500)^2)/100 + 13/100))/(160*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160))))/((x(1)^2*(sin(x(3))^2 - 1))/(tan(x(2))^2 + 1) + (x(1)^2*tan(x(2))*(sin(x(3))^2 - 1))/(tan(x(2))^2 + 1));
f(3) = (x(1)*((981*cos(x(8))*sin(x(7)))/100 - x(6)*(-(x(1)^2*(sin(x(3))^2 - 1))/(tan(x(2))^2 + 1))^(1/2) + x(4)*tan(x(2))*(-(x(1)^2*(sin(x(3))^2 - 1))/(tan(x(2))^2 + 1))^(1/2) + (17563*x(1)^2*cos(x(3))*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((8*x(3))/5 - (6*x(14))/25))/(160*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160)) + (17563*x(1)^2*sin(x(3))*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((7*((11*x(2))/2 + 327/500)^2)/100 + 13/100))/(160*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160))) - (x(1)*sin(x(3))*(x(1)*sin(x(3))*((981*cos(x(8))*sin(x(7)))/100 - x(6)*(-(x(1)^2*(sin(x(3))^2 - 1))/(tan(x(2))^2 + 1))^(1/2) + x(4)*tan(x(2))*(-(x(1)^2*(sin(x(3))^2 - 1))/(tan(x(2))^2 + 1))^(1/2) + (17563*x(1)^2*cos(x(3))*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((8*x(3))/5 - (6*x(14))/25))/(160*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160)) + (17563*x(1)^2*sin(x(3))*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((7*((11*x(2))/2 + 327/500)^2)/100 + 13/100))/(160*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160))) - (-(x(1)^2*(sin(x(3))^2 - 1))/(tan(x(2))^2 + 1))^(1/2)*((981*sin(x(8)))/100 - (981*x(12))/100 - x(6)*x(1)*sin(x(3)) + x(5)*tan(x(2))*(-(x(1)^2*(sin(x(3))^2 - 1))/(tan(x(2))^2 + 1))^(1/2) + (17563*x(1)^2*sin(x(2))*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((3947*x(2))/650 + (248*x(11))/325 + (79373*pi)/234000 + (15376*x(5))/(625*x(1))))/(160*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160)) + (17563*x(1)^2*cos(x(2))*sin(x(3))*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((8*x(3))/5 - (6*x(14))/25))/(160*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160)) - (17563*x(1)^2*cos(x(2))*cos(x(3))*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((7*((11*x(2))/2 + 327/500)^2)/100 + 13/100))/(160*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160))) + tan(x(2))*(-(x(1)^2*(sin(x(3))^2 - 1))/(tan(x(2))^2 + 1))^(1/2)*((981*cos(x(7))*cos(x(8)))/100 + x(5)*(-(x(1)^2*(sin(x(3))^2 - 1))/(tan(x(2))^2 + 1))^(1/2) - x(4)*x(1)*sin(x(3)) + (17563*x(1)^2*cos(x(2))*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((3947*x(2))/650 + (248*x(11))/325 + (79373*pi)/234000 + (15376*x(5))/(625*x(1))))/(160*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160)) - (17563*x(1)^2*sin(x(2))*sin(x(3))*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((8*x(3))/5 - (6*x(14))/25))/(160*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160)) + (17563*x(1)^2*cos(x(3))*sin(x(2))*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((7*((11*x(2))/2 + 327/500)^2)/100 + 13/100))/(160*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160)))))/(x(1)^2*sin(x(3))^2 - (x(1)^2*(sin(x(3))^2 - 1))/(tan(x(2))^2 + 1) - (x(1)^2*tan(x(2))^2*(sin(x(3))^2 - 1))/(tan(x(2))^2 + 1))^(1/2))/(x(1)*(- x(1)^2*sin(x(3))^2 + x(1)^2)^(1/2));
f(4) = (46464382374588000220125*x(1)^2*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((5327*x(3))/200 + (2283*x(13))/200 - (8371*x(14))/2000 + (276243*x(4))/(200*x(1)) - (25113*x(6))/(40*x(1))))/(37778931862957161709568*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160)) - (73367116026106252954079*x(5)*x(6))/75557863725914323419136;
f(5) = (246513127265788095207*x(12))/4722366482869645213696 + (36990728989196953634463*x(4)*x(6))/37778931862957161709568 + (16717287763603297177125*x(1)^2*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((23064*x(2))/1625 + (30752*x(11))/1625 - (22103*pi)/73125 + (107333389199195577*x(5))/(175921860444160*x(1)) + 1947/500))/(18889465931478580854784*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160));
f(6) = (39364648287149037864375*x(1)^2*(1 - (13*x(10))/576300)^(2959813833481473/562949953421312)*((47943*x(14))/4000 + (761*x(3)*((8601241708833989*x(2))/2251799813685248 - 1))/40 - (426921*x(4))/(2000*x(1)) + (577599*x(6))/(400*x(1))))/(75557863725914323419136*((32824283573950349*x(10))/17592186044416000 - 14551257402821220099/175921860444160)) - (3118301144848121130375*x(4)*x(5))/18889465931478580854784;
f(7) = x(4) + (x(6)*cos(x(7))*sin(x(8)))/cos(x(8)) + (x(5)*sin(x(7))*sin(x(8)))/cos(x(8));
f(8) = x(5)*cos(x(7)) - x(6)*sin(x(7));
f(9) = (x(6)*cos(x(7)))/cos(x(8)) + (x(5)*sin(x(7)))/cos(x(8));
f(10) = sin(x(8))*(-(x(1)^2*(sin(x(3))^2 - 1))/(tan(x(2))^2 + 1))^(1/2) - x(1)*sin(x(3))*cos(x(8))*sin(x(7)) - cos(x(7))*tan(x(2))*cos(x(8))*(-(x(1)^2*(sin(x(3))^2 - 1))/(tan(x(2))^2 + 1))^(1/2);
f(11) = x(1)-150;
f(12) = x(7);
f(13) = x(9);
f(14) = x(10)-5000;
end