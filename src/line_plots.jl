export
    plot,
    plot_trajectory!

function Plots.plot(X::Vector{SVector{N,Float64}}, inds=1:N; kwargs...) where N
    A = Array(hcat(X...))[inds,:]
    plot(A'; kwargs...)
end

function Plots.plot(t, X::Vector{SVector{N,Float64}}, inds=1:N; kwargs...) where N
    A = Array(hcat(X...))[inds,:]
    plot(t[1:length(X)], A'; kwargs...)
end

function Plots.plot!(X::Vector{SVector{N,Float64}}, inds=1:N; kwargs...) where N
    A = Array(hcat(X...))[inds,:]
    plot!(A'; kwargs...)
end

function plot_trajectory!(X::Vector; kwargs...)
    x = [x[1] for x in X]
    y = [x[2] for x in X]
    Plots.plot!(x,y; kwargs...)
end
