require 'orocos'
require 'orocos/log'
require 'rock/bundle'
require 'vizkit'
require 'transformer/runtime'
require_relative 'visualize'

include Orocos

## Setup replay
unless ARGV.length >= 1
	puts "Supply path to replay!"
	exit
end

log = Orocos::Log::Replay.open(ARGV[0])
log.use_sample_time = false

## Initialize orocos ##
Orocos.initialize

# track only needed ports
scan_ports = log.find_all_output_ports("/base/samples/LaserScan", "scans")
scan_ports.each do |port|
	port.tracked = true
end

#odometry_ports = log.find_all_output_ports("/base/samples/RigidBodyState_m", "odometry")
#odometry_ports.each do |port|
#	port.tracked = true
#end

state_ports = log.find_all_output_ports("/int32_t", "state")
state_ports.each do |port|
    port.tracked = true
end

log.transformer_broadcaster.track(false)
log.transformer_broadcaster.rename("foo")

## Execute the task 'message_producer::Task' ##
Orocos.run 'laserscan_fusion::MergeTwoScans' => 'fusion',
           'slam3d::PointcloudMapper2D' => 'mapper' do

	Bundles.transformer.start_broadcaster do

		############################################################################
		## Configure the scan fusion ##
		fusion = Orocos.name_service.get 'fusion'
		scan_ports.each do |port|
			if port.task.name == "aila-control/hokuyo_front"
				port.connect_to fusion.scan1
				frontViz = Vizkit.default_loader.LaserScanVisualization
				port.connect_to do |data,_|
					frontViz.updateData(data)
				end
				frontViz.frame = "laser1"
			elsif port.task.name == "aila-control/hokuyo_rear"
				port.connect_to fusion.scan2
				rearViz = Vizkit.default_loader.LaserScanVisualization
				port.connect_to do |data,_|
					rearViz.updateData(data)
				end
				rearViz.frame = "laser2"
			end
		end
		Orocos.transformer.load_conf("aila_tf.rb")
		Orocos.transformer.setup(fusion)
		fusion.configure
	
		############################################################################
		## Configure the mapper ##
		mapper = Orocos.name_service.get 'mapper'
		mapper.scan_resolution = 0.0
		mapper.map_resolution = 0.01
		mapper.map_outlier_radius = 0.2
		mapper.map_outlier_neighbors = 2
		mapper.neighbor_radius = 2.0
		mapper.min_translation = 0.1
		mapper.min_rotation = 0.25
		mapper.use_odometry = false
		mapper.add_odometry_edges = true
		mapper.scan_period = 0.1
		mapper.log_level = 1
		mapper.map_frame = "world_osg"
		mapper.robot_frame = "body"
		mapper.robot_name = "Aila"
		mapper.use_colors_as_viewpoints = true
		mapper.map_publish_rate = 0
	
		mapper.gicp_config do |c|
			c.max_correspondence_distance = 0.25
			c.max_fitness_score = 30
			c.point_cloud_density = 0.1
			c.maximum_iterations = 20
		end
	
		mapper.grid_size_x = 20
		mapper.grid_size_y = 20
		mapper.grid_offset_x = -10
		mapper.grid_offset_y = -10
		mapper.grid_min_z = -5;
		mapper.grid_max_z = 5;
		mapper.grid_resolution = 0.05
		mapper.configure

		fusion.cloud.connect_to mapper.scan,              :type => :buffer, :size => 10

		############################################################################
	
		fusion.start
		mapper.start
	
		############################################################################
		## Start the replay ##
		Vizkit.control log

#		Vizkit.display fusion.cloud
#		Vizkit.display mapper.cloud
		Vizkit.display mapper.envire_map

		begin
			Vizkit.exec
		rescue Interrupt => e
			mapper.stop
			mapper.cleanup
			fusion.stop
			fusion.cleanup
		end

	end
end
