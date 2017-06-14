require 'orocos'
require 'rock/bundle'
# require 'vizkit'
include Orocos
require "readline"

puts("PLY Path is #{ARGV[0]}")

Bundles.initialize

Bundles.run 'envire_exporters::MLSMapKalmanExporter' => "export", :gdb => false, :valgrind => false do

    exporter = Bundles.get 'export'

    exporter.path = ARGV[0]

    exporter.configure

    exporter.start
    
    sleep(5)
    
    exporter.writeMap
    
    Readline.readline()
    
end
