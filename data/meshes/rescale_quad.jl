using Printf

function rescale_mesh(infile, outfile; scale=1.0f0)
    scale = Float32(scale) 
    fid = open(infile)
    out = open(outfile, write=true)
    for line = eachline(fid)
        if length(line) > 0 && line[1] == 'v'  && line[2] != 't'
            ldata = split(line, " ")
            data = parse.(Float32, ldata[2:4]) * scale
            scaled_line = @sprintf("%s %.6f %.6f %.6f", ldata[1], data...)
            println(out, scaled_line)
        else
            println(out, line)
        end
    end
    close(out)
    close(fid)
end
rescale_mesh(
    joinpath(@__DIR__,"quadrotor_base.obj"),
    joinpath(@__DIR__,"quadrotor_scaled.obj"),
    scale = 0.085
)

rescale_mesh(
    joinpath(@__DIR__,"piper","piper_pa18.obj"),
    joinpath(@__DIR__,"piper","piper_scaled.obj"),
    scale = 0.10
)