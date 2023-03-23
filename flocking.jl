using GLMakie
using DataStructures: CircularBuffer
using Makie.GeometryBasics
using LinearAlgebra
using Colors


empty_theme = Theme(
                  Axis = (
                      backgroundcolor = :gray15,
                      leftspinevisible = false,
                      rightspinevisible = false,
                      bottomspinevisible = false,
                      topspinevisible = false,
                      xticklabelsvisible = false,
                      yticklabelsvisible = false,
                      xgridcolor = :transparent,
                      ygridcolor = :transparent,
                      xminorticksvisible = false,
                      yminorticksvisible = false,
                      xticksvisible = false,
                      yticksvisible = false,
                      xautolimitmargin = (0.0,0.0),
                      yautolimitmargin = (0.0,0.0),
                      resolution = (1000, 1000),
                      #figure_padding = 0,

                  )
              )

fig = Figure()


set_theme!(empty_theme)
ax = Axis(fig[1,1], aspect=1)
xlims!(ax, -1, 1) # as vector
ylims!(ax, -1, 1)
Makie.deactivate_interaction!(ax, :rectanglezoom)

screen = display(fig)
resize!(screen, 1500, 1500)
scene = Scene(camera = campixel!)

##########################################################

function rotate(x, α)
    y = Array{Float64}(undef, 2)
    y[1] = x[1] * cos(α) - x[2] * sin(α)
    y[2] = x[1] * sin(α) + x[2] * cos(α)
    return y
end

function normalize_smart(vec)
    return norm(vec) != 0 ? normalize(vec) : [0.,0.]
end

##########################################################

mutable struct Boid
    pos
    vel
    avatar
    new_pos
    new_vel
    group_color
end

function Boid(pos, vel, group_color)
    scale = 0.020
    dir = scale*vel/sqrt(vel[1]^2 + vel[2]^2)
    avatar = Observable(Polygon(Point2f[pos + dir,
                                        pos + rotate(dir, 3/4 * π),
                                        pos + rotate(dir, -3/4 * π)]))
    return Boid(pos, vel, avatar, nothing, nothing, group_color)
end

function apply!(boid::Boid, force)
    max_vel = 3.0
    max_force =3.0
    Δt = 0.005

    #desired_vel = normalize([0., 0.] - boid.pos) * max_vel
    #force = normalize(desired_vel - boid.vel) * max_force
    boid.new_vel = boid.vel + Δt * force
    boid.new_pos = boid.pos + Δt * boid.new_vel
end

on(events(scene).mouseposition) do mp
    print(mp)
end

function move!(boid::Boid)
    boid.pos = boid.new_pos
    boid.vel = boid.new_vel
    boid.avatar[] = Boid(boid.pos, boid.vel, boid.group_color).avatar[]
end

function plot!(boid::Boid)
    poly!(boid.avatar, color = (boid.group_color, rand()))#(RGB(0., rand(), rand()), 0.8)) # 
end

struct Flock
    boids
    group_color
end

function Flock(n::Integer, group_color)
    return Flock([Boid(2*(rand(2).-0.5), 2*(rand(2).-0.5), group_color) for _ in 1:n], group_color)
end

#function think!(flock::Flock)
#    max_vel = 3.0
#    max_force =3.0
#    Δt = 0.005
#
#    apply!.(flock.boids, forces)
#end

function think!(flock::Flock, external_force, others, others_scale)
    max_vel = 6.0
    max_force = 5.
    Δt = 0.009
    cohere_scale = 1.0 #      0.5    0.8
    align_scale = 0.2  #      0.2    0.4
    separate_scale = 2.2  #   2.2    2.8
    #others_scale = 5.
    border_scale = .0 # 0.0

    for boid in flock.boids
        avg_vel = [0., 0.]
        avg_pos = [0., 0.]
        away = [0., 0.]
        n_neighbours = 0
        n_too_close = 0
        for neighbour in flock.boids
            if (norm(neighbour.pos - boid.pos) <= 0.5)
                avg_vel += neighbour.vel
                avg_pos += neighbour.pos
                n_neighbours += 1
            end
            if (norm(neighbour.pos - boid.pos) <= 0.065)
                away += boid.pos - neighbour.pos
                n_too_close += 1
            end
        end

        n_others = 0
        others_vel = [0.,0.]
        for other in others.boids
            if (norm(other.pos - boid.pos) <= 0.20)
                others_vel += boid.pos - other.pos
                n_others += 1
                #print("IM here")
            end
        end

        if n_neighbours != 0
            avg_vel /= n_neighbours
            avg_pos /= n_neighbours
        else
            avg_vel = [0., 0.]
            avg_pos = [0., 0.]
        end

        if n_too_close != 0
            away /= n_too_close
        else
            away = [0., 0.]
        end

        #if n_others != 0
        #    other_vel /= n_others
        #else
        #    away = [0., 0.]
        #end

        others_force = [0., 0.]
        if n_others != 0
            others_desired_vel = normalize_smart(others_vel) * max_vel
            others_force = normalize_smart(others_desired_vel-boid.vel) * others_scale
        end
        #print(others_force)

        if norm(away) == 0
            away_force = [0., 0.]
        else
            desired_vel_away = normalize(away) * max_vel
            away_force = normalize(desired_vel_away - boid.vel) * separate_scale
        end

        #avg_vel = n_neighbours != 0 ? avg_vel/n_neighbours : [0,0]

        # target
        if norm(avg_pos - boid.pos) == 0
            cohere_force = [0., 0.]
        else
            desired_vel = normalize(avg_pos - boid.pos) * max_vel
            cohere_force = normalize(desired_vel - boid.vel) * cohere_scale
        end

        # align
        if norm(avg_vel - boid.vel) == 0
            align_force = [0., 0.]
        else
            align_force = normalize(avg_vel - boid.vel) * align_scale
        end

        # border

        border_force = [0., 0.]
        border_desired_velocity = [0.,0.]
        border_desired_velocity += (boid.pos[1] <= -1 ? [1. ,  0.] : [0., 0.])
        border_desired_velocity += (boid.pos[1] >= +1 ? [-1.,  0.] : [0., 0.])
        border_desired_velocity += (boid.pos[2] <= -1 ? [0. ,  1.] : [0., 0.])
        border_desired_velocity += (boid.pos[2] >= +1 ? [0. , -1.] : [0., 0.])

        border_desired_velocity = normalize_smart(border_desired_velocity) * max_vel
        border_force = normalize_smart(border_desired_velocity - boid.vel) * border_scale

        # total force
        force = align_force + cohere_force + away_force + border_force + others_force + external_force

        if norm(force) != 0
            force = min(norm(force), max_force) * normalize(force)
        else
            force = [0., 0.]
        end

        world_size = 2.2

        boid.new_vel = boid.vel + Δt * force
        boid.new_pos = mod.(boid.pos + Δt * boid.new_vel + [world_size/2, world_size/2], world_size) - [world_size/2, world_size/2]
    end

end

function move!(flock::Flock, external_force, others, others_scale)
    think!(flock, external_force, others, others_scale)
    move!.(flock.boids)
end

function plot!(flock::Flock)
    plot!.(flock.boids)
end


#itches = [Boid(2*(rand(2).-0.5), 2*(rand(2).-0.5)) for _ in 1:30]
flock1 = Flock(200, :white) #200
flock2 = Flock(200, :green)

#scatter!(Point2f(0.0, 0.0), markersize = 20, color = :green)
plot!(flock1)
plot!(flock2)
migration_dir1 = [0.5, 0.3]
while true#for i in 1:2000
    #think!(flock)
    global migration_dir1 = rotate(migration_dir1, 0.02*(rand()-0.5))
    move!(flock1, migration_dir1, flock2, 0.2)
    move!(flock2, [0.2, 0.], flock1, 2.0)
    #print(events(scene).mouseposition[])
    sleep(0.01)
end




nothing
