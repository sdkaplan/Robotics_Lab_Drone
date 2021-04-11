 // License: Apache 2.0. See LICENSE file in root directory.
 // Copyright(c) 2017 Intel Corporation. All Rights Reserved.
 
 #ifndef LIBREALSENSE_RS2_PROCESSING_HPP
 #define LIBREALSENSE_RS2_PROCESSING_HPP
 
 #include "rs_types.hpp"
 #include "rs_frame.hpp"
 #include "rs_options.hpp"
 
 namespace rs2
 {
     class frame_source
     {
     public:
         frame allocate_video_frame(const stream_profile& profile,
             const frame& original,
             int new_bpp = 0,
             int new_width = 0,
             int new_height = 0,
             int new_stride = 0,
             rs2_extension frame_type = RS2_EXTENSION_VIDEO_FRAME) const
         {
             rs2_error* e = nullptr;
             auto result = rs2_allocate_synthetic_video_frame(_source, profile.get(),
                 original.get(), new_bpp, new_width, new_height, new_stride, frame_type, &e);
             error::handle(e);
             return result;
         }
 
         frame allocate_motion_frame(const stream_profile& profile,
             const frame& original,
             rs2_extension frame_type = RS2_EXTENSION_MOTION_FRAME) const
         {
             rs2_error* e = nullptr;
             auto result = rs2_allocate_synthetic_motion_frame(_source, profile.get(),
                 original.get(), frame_type, &e);
             error::handle(e);
             return result;
         }
 
         frame allocate_points(const stream_profile& profile,
             const frame& original) const
         {
             rs2_error* e = nullptr;
             auto result = rs2_allocate_points(_source, profile.get(), original.get(), &e);
             error::handle(e);
             return result;
         }
 
         frame allocate_composite_frame(std::vector<frame> frames) const
         {
             rs2_error* e = nullptr;
 
             std::vector<rs2_frame*> refs(frames.size(), (rs2_frame*)nullptr);
             for (size_t i = 0; i < frames.size(); i++)
                 std::swap(refs[i], frames[i].frame_ref);
 
             auto result = rs2_allocate_composite_frame(_source, refs.data(), (int)refs.size(), &e);
             error::handle(e);
             return result;
         }
         void frame_ready(frame result) const
         {
             rs2_error* e = nullptr;
             rs2_synthetic_frame_ready(_source, result.get(), &e);
             error::handle(e);
             result.frame_ref = nullptr;
         }
 
         rs2_source* _source;
     private:
         template<class T>
         friend class frame_processor_callback;
 
         frame_source(rs2_source* source) : _source(source) {}
         frame_source(const frame_source&) = delete;
 
     };
 
     template<class T>
     class frame_processor_callback : public rs2_frame_processor_callback
     {
         T on_frame_function;
     public:
         explicit frame_processor_callback(T on_frame) : on_frame_function(on_frame) {}
 
         void on_frame(rs2_frame* f, rs2_source * source) override
         {
             frame_source src(source);
             frame frm(f);
             on_frame_function(std::move(frm), src);
         }
 
         void release() override { delete this; }
     };
 
     class frame_queue
     {
     public:
         explicit frame_queue(unsigned int capacity, bool keep_frames = false) : _capacity(capacity), _keep(keep_frames)
         {
             rs2_error* e = nullptr;
             _queue = std::shared_ptr<rs2_frame_queue>(
                 rs2_create_frame_queue(capacity, &e),
                 rs2_delete_frame_queue);
             error::handle(e);
         }
 
         frame_queue() : frame_queue(1) {}
 
         void enqueue(frame f) const
         {
             if (_keep) f.keep();
             rs2_enqueue_frame(f.frame_ref, _queue.get()); // noexcept
             f.frame_ref = nullptr; // frame has been essentially moved from
         }
 
         frame wait_for_frame(unsigned int timeout_ms = 5000) const
         {
             rs2_error* e = nullptr;
             auto frame_ref = rs2_wait_for_frame(_queue.get(), timeout_ms, &e);
             error::handle(e);
             return{ frame_ref };
         }
 
         template<typename T>
         typename std::enable_if<std::is_base_of<rs2::frame, T>::value, bool>::type poll_for_frame(T* output) const
         {
             rs2_error* e = nullptr;
             rs2_frame* frame_ref = nullptr;
             auto res = rs2_poll_for_frame(_queue.get(), &frame_ref, &e);
             error::handle(e);
             frame f{ frame_ref };
             if (res) *output = f;
             return res > 0;
         }
 
         template<typename T>
         typename std::enable_if<std::is_base_of<rs2::frame, T>::value, bool>::type try_wait_for_frame(T* output, unsigned int timeout_ms = 5000) const
         {
             rs2_error* e = nullptr;
             rs2_frame* frame_ref = nullptr;
             auto res = rs2_try_wait_for_frame(_queue.get(), timeout_ms, &frame_ref, &e);
             error::handle(e);
             frame f{ frame_ref };
             if (res) *output = f;
             return res > 0;
         }
         void operator()(frame f) const
         {
             enqueue(std::move(f));
         }
         size_t capacity() const { return _capacity; }
 
         bool keep_frames() const { return _keep; }
 
     private:
         std::shared_ptr<rs2_frame_queue> _queue;
         size_t _capacity;
         bool _keep;
     };
 
     class processing_block : public options
     {
     public:
         using options::supports;
 
         template<class S>
         void start(S on_frame)
         {
             rs2_error* e = nullptr;
             rs2_start_processing(get(), new frame_callback<S>(on_frame), &e);
             error::handle(e);
         }
         template<class S>
         S& operator>>(S& on_frame)
         {
             start(on_frame);
             return on_frame;
         }
         void invoke(frame f) const
         {
             rs2_frame* ptr = nullptr;
             std::swap(f.frame_ref, ptr);
 
             rs2_error* e = nullptr;
             rs2_process_frame(get(), ptr, &e);
             error::handle(e);
         }
         processing_block(std::shared_ptr<rs2_processing_block> block)
             : options((rs2_options*)block.get()), _block(block)
         {
         }
 
         template<class S>
         processing_block(S processing_function)
         {
             rs2_error* e = nullptr;
             _block = std::shared_ptr<rs2_processing_block>(
                 rs2_create_processing_block(new frame_processor_callback<S>(processing_function), &e),
                 rs2_delete_processing_block);
             options::operator=(_block);
             error::handle(e);
         }
 
         operator rs2_options*() const { return (rs2_options*)get(); }
         rs2_processing_block* get() const { return _block.get(); }
 
         bool supports(rs2_camera_info info) const
         {
             rs2_error* e = nullptr;
             auto is_supported = rs2_supports_processing_block_info(_block.get(), info, &e);
             error::handle(e);
             return is_supported > 0;
         }
 
         const char* get_info(rs2_camera_info info) const
         {
             rs2_error* e = nullptr;
             auto result = rs2_get_processing_block_info(_block.get(), info, &e);
             error::handle(e);
             return result;
         }
     protected:
         void register_simple_option(rs2_option option_id, option_range range) {
             rs2_error * e = nullptr;
             rs2_processing_block_register_simple_option(_block.get(), option_id,
                     range.min, range.max, range.step, range.def, &e);
             error::handle(e);
         }
         std::shared_ptr<rs2_processing_block> _block;
     };
 
     class filter : public processing_block, public filter_interface
     {
     public:
         rs2::frame process(rs2::frame frame) const override
         {
             invoke(frame);
             rs2::frame f;
             if (!_queue.poll_for_frame(&f))
                 throw std::runtime_error("Error occured during execution of the processing block! See the log for more info");
             return f;
         }
 
         filter(std::shared_ptr<rs2_processing_block> block, int queue_size = 1)
             : processing_block(block),
             _queue(queue_size)
         {
             start(_queue);
         }
 
         template<class S>
         filter(S processing_function, int queue_size = 1) :
             processing_block(processing_function),
             _queue(queue_size)
         {
             start(_queue);
         }
 
 
         frame_queue get_queue() { return _queue; }
         rs2_processing_block* get() const { return _block.get(); }
 
         template<class T>
         bool is() const
         {
             T extension(*this);
             return extension;
         }
 
         template<class T>
         T as() const
         {
             T extension(*this);
             return extension;
         }
 
         operator bool() const { return _block.get() != nullptr; }
     protected:
         frame_queue _queue;
     };
 
     class pointcloud : public filter
     {
     public:
         pointcloud() : filter(init(), 1) {}
 
         pointcloud(rs2_stream stream, int index = 0) : filter(init(), 1)
         {
             set_option(RS2_OPTION_STREAM_FILTER, float(stream));
             set_option(RS2_OPTION_STREAM_INDEX_FILTER, float(index));
         }
         points calculate(frame depth) const
         {
             auto res = process(depth);
             if (res.as<points>())
                 return res;
 
             if (auto set = res.as <frameset>())
             {
                 for (auto f : set)
                 {
                     if(f.as<points>())
                         return f;
                 }
             }
             throw std::runtime_error("Error occured during execution of the processing block! See the log for more info");
         }
         void map_to(frame mapped)
         {
             set_option(RS2_OPTION_STREAM_FILTER, float(mapped.get_profile().stream_type()));
             set_option(RS2_OPTION_STREAM_FORMAT_FILTER, float(mapped.get_profile().format()));
             set_option(RS2_OPTION_STREAM_INDEX_FILTER, float(mapped.get_profile().stream_index()));
             process(mapped);
         }
 
     protected:
         pointcloud(std::shared_ptr<rs2_processing_block> block) : filter(block, 1) {}
 
     private:
         friend class context;
 
         std::shared_ptr<rs2_processing_block> init()
         {
             rs2_error* e = nullptr;
 
             auto block = std::shared_ptr<rs2_processing_block>(
                 rs2_create_pointcloud(&e),
                 rs2_delete_processing_block);
 
             error::handle(e);
 
             // Redirect options API to the processing block
             //options::operator=(pb);
             return block;
         }
     };
 
     class yuy_decoder : public filter
     {
     public:
         yuy_decoder() : filter(init(), 1) { }
 
     protected:
         yuy_decoder(std::shared_ptr<rs2_processing_block> block) : filter(block, 1) {}
 
     private:
         std::shared_ptr<rs2_processing_block> init()
         {
             rs2_error* e = nullptr;
             auto block = std::shared_ptr<rs2_processing_block>(
                 rs2_create_yuy_decoder(&e),
                 rs2_delete_processing_block);
             error::handle(e);
 
             return block;
         }
     };
 
   class threshold_filter : public filter
     {
     public:
         threshold_filter(float min_dist = 0.15f, float max_dist = 4.f)
             : filter(init(), 1)
         {
             set_option(RS2_OPTION_MIN_DISTANCE, min_dist);
             set_option(RS2_OPTION_MAX_DISTANCE, max_dist);
         }
 
         threshold_filter(filter f) : filter(f)
         {
             rs2_error* e = nullptr;
             if (!rs2_is_processing_block_extendable_to(f.get(), RS2_EXTENSION_THRESHOLD_FILTER, &e) && !e)
             {
                 _block.reset();
             }
             error::handle(e);
         }
 
     protected:
         threshold_filter(std::shared_ptr<rs2_processing_block> block) : filter(block, 1) {}
 
     private:
         std::shared_ptr<rs2_processing_block> init()
         {
             rs2_error* e = nullptr;
             auto block = std::shared_ptr<rs2_processing_block>(
                 rs2_create_threshold(&e),
                 rs2_delete_processing_block);
             error::handle(e);
 
             return block;
         }
     };
 
     class units_transform : public filter
     {
     public:
         units_transform() : filter(init(), 1) {}
 
     protected:
         units_transform(std::shared_ptr<rs2_processing_block> block) : filter(block, 1) {}
 
     private:
         std::shared_ptr<rs2_processing_block> init()
         {
             rs2_error* e = nullptr;
             auto block = std::shared_ptr<rs2_processing_block>(
                 rs2_create_units_transform(&e),
                 rs2_delete_processing_block);
             error::handle(e);
 
             return block;
         }
     };
 
     class asynchronous_syncer : public processing_block
     {
     public:
         asynchronous_syncer() : processing_block(init()) {}
 
     private:
         std::shared_ptr<rs2_processing_block> init()
         {
             rs2_error* e = nullptr;
             auto block = std::shared_ptr<rs2_processing_block>(
                 rs2_create_sync_processing_block(&e),
                 rs2_delete_processing_block);
 
             error::handle(e);
             return block;
         }
     };
 
     class syncer
     {
     public:
         syncer(int queue_size = 1)
             :_results(queue_size)
         {
             _sync.start(_results);
         }
 
         frameset wait_for_frames(unsigned int timeout_ms = 5000) const
         {
             return frameset(_results.wait_for_frame(timeout_ms));
         }
 
         bool poll_for_frames(frameset* fs) const
         {
             frame result;
             if (_results.poll_for_frame(&result))
             {
                 *fs = frameset(result);
                 return true;
             }
             return false;
         }
 
         bool try_wait_for_frames(frameset* fs, unsigned int timeout_ms = 5000) const
         {
             frame result;
             if (_results.try_wait_for_frame(&result, timeout_ms))
             {
                 *fs = frameset(result);
                 return true;
             }
             return false;
         }
 
         void operator()(frame f) const
         {
             _sync.invoke(std::move(f));
         }
     private:
         asynchronous_syncer _sync;
         frame_queue _results;
     };
 
     class align : public filter
     {
     public:
         align(rs2_stream align_to) : filter(init(align_to), 1) {}
 
         using filter::process;
 
         frameset process(frameset frames)
         {
             return filter::process(frames);
         }
 
     protected:
         align(std::shared_ptr<rs2_processing_block> block) : filter(block, 1) {}
 
     private:
         friend class context;
         std::shared_ptr<rs2_processing_block> init(rs2_stream align_to)
         {
             rs2_error* e = nullptr;
             auto block = std::shared_ptr<rs2_processing_block>(
                 rs2_create_align(align_to, &e),
                 rs2_delete_processing_block);
             error::handle(e);
 
             return block;
         }
     };
 
     class colorizer : public filter
     {
     public:
         colorizer() : filter(init(), 1) { }
         colorizer(float color_scheme) : filter(init(), 1)
         {
             set_option(RS2_OPTION_COLOR_SCHEME, float(color_scheme));
         }
         video_frame colorize(frame depth) const
         {
             return process(depth);
         }
 
     protected:
         colorizer(std::shared_ptr<rs2_processing_block> block) : filter(block, 1) {}
 
     private:
         std::shared_ptr<rs2_processing_block> init()
         {
             rs2_error* e = nullptr;
             auto block = std::shared_ptr<rs2_processing_block>(
                 rs2_create_colorizer(&e),
                 rs2_delete_processing_block);
             error::handle(e);
 
             // Redirect options API to the processing block
             //options::operator=(pb);
 
             return block;
         }
     };
 
     class decimation_filter : public filter
     {
     public:
         decimation_filter() : filter(init(), 1) {}
         decimation_filter(float magnitude) : filter(init(), 1)
         {
             set_option(RS2_OPTION_FILTER_MAGNITUDE, magnitude);
         }
 
         decimation_filter(filter f) : filter(f)
         {
              rs2_error* e = nullptr;
              if (!rs2_is_processing_block_extendable_to(f.get(), RS2_EXTENSION_DECIMATION_FILTER, &e) && !e)
              {
                  _block.reset();
              }
              error::handle(e);
         }
 
     private:
         friend class context;
 
         std::shared_ptr<rs2_processing_block> init()
         {
             rs2_error* e = nullptr;
             auto block = std::shared_ptr<rs2_processing_block>(
                 rs2_create_decimation_filter_block(&e),
                 rs2_delete_processing_block);
             error::handle(e);
 
             // Redirect options API to the processing block
             //options::operator=(this);
 
             return block;
         }
     };
 
     class temporal_filter : public filter
     {
     public:
         temporal_filter() : filter(init(), 1) {}
         temporal_filter(float smooth_alpha, float smooth_delta, int persistence_control) : filter(init(), 1)
         {
             set_option(RS2_OPTION_HOLES_FILL, float(persistence_control));
             set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, float(smooth_alpha));
             set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, float(smooth_delta));
         }
 
         temporal_filter(filter f) :filter(f)
         {
             rs2_error* e = nullptr;
             if (!rs2_is_processing_block_extendable_to(f.get(), RS2_EXTENSION_TEMPORAL_FILTER, &e) && !e)
             {
                 _block.reset();
             }
             error::handle(e);
         }
     private:
         friend class context;
 
         std::shared_ptr<rs2_processing_block> init()
         {
             rs2_error* e = nullptr;
             auto block = std::shared_ptr<rs2_processing_block>(
                 rs2_create_temporal_filter_block(&e),
                 rs2_delete_processing_block);
             error::handle(e);
 
             // Redirect options API to the processing block
             //options::operator=(pb);
 
             return block;
         }
     };
 
     class spatial_filter : public filter
     {
     public:
         spatial_filter() : filter(init(), 1) { }
 
         spatial_filter(float smooth_alpha, float smooth_delta, float magnitude, float hole_fill) : filter(init(), 1)
         {
             set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, float(smooth_alpha));
             set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, float(smooth_delta));
             set_option(RS2_OPTION_FILTER_MAGNITUDE, magnitude);
             set_option(RS2_OPTION_HOLES_FILL, hole_fill);
         }
 
         spatial_filter(filter f) :filter(f)
         {
             rs2_error* e = nullptr;
             if (!rs2_is_processing_block_extendable_to(f.get(), RS2_EXTENSION_SPATIAL_FILTER, &e) && !e)
             {
                 _block.reset();
             }
             error::handle(e);
         }
     private:
         friend class context;
 
         std::shared_ptr<rs2_processing_block> init()
         {
             rs2_error* e = nullptr;
             auto block = std::shared_ptr<rs2_processing_block>(
                 rs2_create_spatial_filter_block(&e),
                 rs2_delete_processing_block);
             error::handle(e);
 
             // Redirect options API to the processing block
             //options::operator=(pb);
 
             return block;
         }
     };
 
     class disparity_transform : public filter
     {
     public:
         disparity_transform(bool transform_to_disparity = true) : filter(init(transform_to_disparity), 1) { }
 
         disparity_transform(filter f) :filter(f)
         {
             rs2_error* e = nullptr;
             if (!rs2_is_processing_block_extendable_to(f.get(), RS2_EXTENSION_DISPARITY_FILTER, &e) && !e)
             {
                 _block.reset();
             }
             error::handle(e);
         }
     private:
         friend class context;
         std::shared_ptr<rs2_processing_block> init(bool transform_to_disparity)
         {
             rs2_error* e = nullptr;
             auto block = std::shared_ptr<rs2_processing_block>(
                 rs2_create_disparity_transform_block(uint8_t(transform_to_disparity), &e),
                 rs2_delete_processing_block);
             error::handle(e);
 
             // Redirect options API to the processing block
             //options::operator=(pb);
 
             return block;
         }
     };
 
     class zero_order_invalidation : public filter
     {
     public:
         zero_order_invalidation() : filter(init())
         {}
 
         zero_order_invalidation(filter f) :filter(f)
         {
             rs2_error* e = nullptr;
             if (!rs2_is_processing_block_extendable_to(f.get(), RS2_EXTENSION_ZERO_ORDER_FILTER, &e) && !e)
             {
                 _block.reset();
             }
             error::handle(e);
         }
 
     private:
         friend class context;
 
         std::shared_ptr<rs2_processing_block> init()
         {
             rs2_error* e = nullptr;
             auto block = std::shared_ptr<rs2_processing_block>(
                 rs2_create_zero_order_invalidation_block(&e),
                 rs2_delete_processing_block);
             error::handle(e);
 
             return block;
         }
     };
 
     class depth_huffman_decoder : public filter
     {
     public:
         depth_huffman_decoder() : filter(init())
         {}
 
         depth_huffman_decoder(filter f) :filter(f)
         {
             rs2_error* e = nullptr;
             if (!rs2_is_processing_block_extendable_to(f.get(), RS2_EXTENSION_DEPTH_HUFFMAN_DECODER, &e) && !e)
             {
                 _block.reset();
             }
             error::handle(e);
         }
 
     private:
         friend class context;
 
         std::shared_ptr<rs2_processing_block> init()
         {
             rs2_error* e = nullptr;
             auto block = std::shared_ptr<rs2_processing_block>(
                 rs2_create_huffman_depth_decompress_block(&e),
                 rs2_delete_processing_block);
             error::handle(e);
 
             return block;
         }
     };
 
     class hole_filling_filter : public filter
     {
     public:
         hole_filling_filter() : filter(init(), 1) {}
 
         hole_filling_filter(int mode) : filter(init(), 1)
         {
             set_option(RS2_OPTION_HOLES_FILL, float(mode));
         }
 
         hole_filling_filter(filter f) :filter(f)
         {
             rs2_error* e = nullptr;
             if (!rs2_is_processing_block_extendable_to(f.get(), RS2_EXTENSION_HOLE_FILLING_FILTER, &e) && !e)
             {
                 _block.reset();
             }
             error::handle(e);
         }
     private:
         friend class context;
 
         std::shared_ptr<rs2_processing_block> init()
         {
             rs2_error* e = nullptr;
             auto block = std::shared_ptr<rs2_processing_block>(
                 rs2_create_hole_filling_filter_block(&e),
                 rs2_delete_processing_block);
             error::handle(e);
 
             // Redirect options API to the processing block
             //options::operator=(_block);
 
             return block;
         }
     };
 
     class rates_printer : public filter
     {
     public:
         rates_printer() : filter(init(), 1) {}
 
     private:
         friend class context;
 
         std::shared_ptr<rs2_processing_block> init()
         {
             rs2_error* e = nullptr;
             auto block = std::shared_ptr<rs2_processing_block>(
                 rs2_create_rates_printer_block(&e),
                 rs2_delete_processing_block);
             error::handle(e);
 
             return block;
         }
     };
 
     class hdr_merge : public filter
     {
     public:
         hdr_merge() : filter(init()) {}
 
         hdr_merge(filter f) :filter(f)
         {
             rs2_error* e = nullptr;
             if (!rs2_is_processing_block_extendable_to(f.get(), RS2_EXTENSION_HDR_MERGE, &e) && !e)
             {
                 _block.reset();
             }
             error::handle(e);
         }
 
     private:
         friend class context;
 
         std::shared_ptr<rs2_processing_block> init()
         {
             rs2_error* e = nullptr;
             auto block = std::shared_ptr<rs2_processing_block>(
                 rs2_create_hdr_merge_processing_block(&e),
                 rs2_delete_processing_block);
             error::handle(e);
 
             return block;
         }
     };
 
     class sequence_id_filter : public filter
     {
     public:
         sequence_id_filter() : filter(init()) {}
 
         sequence_id_filter(float sequence_id) : filter(init(), 1)
         {
             set_option(RS2_OPTION_SEQUENCE_ID, sequence_id);
         }
 
         sequence_id_filter(filter f) :filter(f)
         {
             rs2_error* e = nullptr;
             if (!rs2_is_processing_block_extendable_to(f.get(), RS2_EXTENSION_SEQUENCE_ID_FILTER, &e) && !e)
             {
                 _block.reset();
             }
             error::handle(e);
         }
 
     private:
         friend class context;
 
         std::shared_ptr<rs2_processing_block> init()
         {
             rs2_error* e = nullptr;
             auto block = std::shared_ptr<rs2_processing_block>(
                 rs2_create_sequence_id_filter(&e),
                 rs2_delete_processing_block);
             error::handle(e);
 
             return block;
         }
     };
 }
 #endif // LIBREALSENSE_RS2_PROCESSING_HPP

