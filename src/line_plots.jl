

function Plots.plot(A::Vector{<:SVector{N}}, inds=1:N; kwargs...) where N
    n = length(inds)
    X = zeros(n,length(A))
    for k in eachindex(A)
        X[:,k] = A[k][inds]
    end
    plot(X'; kwargs...)
end

function Plots.plot(t::Vector{<:Real}, A::Vector{<:SVector{N}}, inds=1:N; kwargs...) where N
    n = length(inds)
    X = zeros(n,length(A))
    for k in eachindex(A)
        X[:,k] = A[k][inds]
    end
    plot(t[1:size(X,2)], X'; kwargs...)
end

function Plots.plot!(A::Vector{<:SVector{N}}, inds=1:N; kwargs...) where N
    n = length(inds)
    X = zeros(n,length(A))
    for k in eachindex(A)
        X[:,k] = A[k][inds]
    end
    plot!(X'; kwargs...)
end

function Plots.plot!(t::Vector{<:Real}, A::Vector{<:SVector{N}}, inds=1:N; kwargs...) where N
    n = length(inds)
    X = zeros(n,length(A))
    for k in eachindex(A)
        X[:,k] = A[k][inds]
    end
    plot!(t[1:size(X,2)], X'; kwargs...)
end

function plot_trajectory!(X::Vector; kwargs...)
    x = [x[1] for x in X]
    y = [x[2] for x in X]
    Plots.plot!(x,y; kwargs...)
end
