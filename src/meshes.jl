using Printf
function rescale_mesh(infile, outfile; scale=1.0f0)
    scale = Float32(scale) 
    fid = open(infile)
    out = open(outfile, write=true)
    for line = eachline(fid)
        if length(line) > 0 && line[1] == "v"
            ldata = split(line, " ")
            data = parse.(Float32, ldata[2:4]) * scale
            scaled_line = @sprintf("%s %.6f %.6f %.6f", ldata[1], data...)
            println(out, scaled_line)
            println(scaled_line)
        else
            println(out, line)
        end
    end
    close(out)
    close(fid)
end