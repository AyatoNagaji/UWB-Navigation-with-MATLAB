function[rs]=MYawgn(r,snr,n)
rs=0;
for i=1:n
    rs=rs+awgn(r,snr);
end