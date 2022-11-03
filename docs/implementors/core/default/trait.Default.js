(function() {var implementors = {};
implementors["byteorder"] = [{"text":"impl <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a> for <a class=\"enum\" href=\"byteorder/enum.BigEndian.html\" title=\"enum byteorder::BigEndian\">BigEndian</a>","synthetic":false,"types":["byteorder::BigEndian"]},{"text":"impl <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a> for <a class=\"enum\" href=\"byteorder/enum.LittleEndian.html\" title=\"enum byteorder::LittleEndian\">LittleEndian</a>","synthetic":false,"types":["byteorder::LittleEndian"]}];
implementors["hash32"] = [{"text":"impl <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a> for <a class=\"struct\" href=\"hash32/struct.FnvHasher.html\" title=\"struct hash32::FnvHasher\">Hasher</a>","synthetic":false,"types":["hash32::fnv::Hasher"]},{"text":"impl <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a> for <a class=\"struct\" href=\"hash32/struct.Murmur3Hasher.html\" title=\"struct hash32::Murmur3Hasher\">Hasher</a>","synthetic":false,"types":["hash32::murmur3::Hasher"]},{"text":"impl&lt;H&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a> for <a class=\"struct\" href=\"hash32/struct.BuildHasherDefault.html\" title=\"struct hash32::BuildHasherDefault\">BuildHasherDefault</a>&lt;H&gt; <span class=\"where fmt-newline\">where<br>&nbsp;&nbsp;&nbsp;&nbsp;H: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a> + <a class=\"trait\" href=\"hash32/trait.Hasher.html\" title=\"trait hash32::Hasher\">Hasher</a>,&nbsp;</span>","synthetic":false,"types":["hash32::BuildHasherDefault"]}];
implementors["heapless"] = [{"text":"impl&lt;T, const N:&nbsp;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.64.0/core/primitive.usize.html\">usize</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a> for <a class=\"struct\" href=\"heapless/struct.Deque.html\" title=\"struct heapless::Deque\">Deque</a>&lt;T, N&gt;","synthetic":false,"types":["heapless::deque::Deque"]},{"text":"impl&lt;T, const N:&nbsp;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.64.0/core/primitive.usize.html\">usize</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a> for <a class=\"struct\" href=\"heapless/struct.HistoryBuffer.html\" title=\"struct heapless::HistoryBuffer\">HistoryBuffer</a>&lt;T, N&gt;","synthetic":false,"types":["heapless::histbuf::HistoryBuffer"]},{"text":"impl&lt;K, V, S, const N:&nbsp;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.64.0/core/primitive.usize.html\">usize</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a> for <a class=\"struct\" href=\"heapless/struct.IndexMap.html\" title=\"struct heapless::IndexMap\">IndexMap</a>&lt;K, V, S, N&gt; <span class=\"where fmt-newline\">where<br>&nbsp;&nbsp;&nbsp;&nbsp;K: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/cmp/trait.Eq.html\" title=\"trait core::cmp::Eq\">Eq</a> + <a class=\"trait\" href=\"hash32/trait.Hash.html\" title=\"trait hash32::Hash\">Hash</a>,<br>&nbsp;&nbsp;&nbsp;&nbsp;S: <a class=\"trait\" href=\"hash32/trait.BuildHasher.html\" title=\"trait hash32::BuildHasher\">BuildHasher</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a>,&nbsp;</span>","synthetic":false,"types":["heapless::indexmap::IndexMap"]},{"text":"impl&lt;T, S, const N:&nbsp;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.64.0/core/primitive.usize.html\">usize</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a> for <a class=\"struct\" href=\"heapless/struct.IndexSet.html\" title=\"struct heapless::IndexSet\">IndexSet</a>&lt;T, S, N&gt; <span class=\"where fmt-newline\">where<br>&nbsp;&nbsp;&nbsp;&nbsp;T: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/cmp/trait.Eq.html\" title=\"trait core::cmp::Eq\">Eq</a> + <a class=\"trait\" href=\"hash32/trait.Hash.html\" title=\"trait hash32::Hash\">Hash</a>,<br>&nbsp;&nbsp;&nbsp;&nbsp;S: <a class=\"trait\" href=\"hash32/trait.BuildHasher.html\" title=\"trait hash32::BuildHasher\">BuildHasher</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a>,&nbsp;</span>","synthetic":false,"types":["heapless::indexset::IndexSet"]},{"text":"impl&lt;K, V, const N:&nbsp;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.64.0/core/primitive.usize.html\">usize</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a> for <a class=\"struct\" href=\"heapless/struct.LinearMap.html\" title=\"struct heapless::LinearMap\">LinearMap</a>&lt;K, V, N&gt; <span class=\"where fmt-newline\">where<br>&nbsp;&nbsp;&nbsp;&nbsp;K: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/cmp/trait.Eq.html\" title=\"trait core::cmp::Eq\">Eq</a>,&nbsp;</span>","synthetic":false,"types":["heapless::linear_map::LinearMap"]},{"text":"impl&lt;const N:&nbsp;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.64.0/core/primitive.usize.html\">usize</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a> for <a class=\"struct\" href=\"heapless/struct.String.html\" title=\"struct heapless::String\">String</a>&lt;N&gt;","synthetic":false,"types":["heapless::string::String"]},{"text":"impl&lt;T, const N:&nbsp;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.64.0/core/primitive.usize.html\">usize</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a> for <a class=\"struct\" href=\"heapless/struct.Vec.html\" title=\"struct heapless::Vec\">Vec</a>&lt;T, N&gt;","synthetic":false,"types":["heapless::vec::Vec"]},{"text":"impl&lt;T, K, const N:&nbsp;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.64.0/core/primitive.usize.html\">usize</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a> for <a class=\"struct\" href=\"heapless/binary_heap/struct.BinaryHeap.html\" title=\"struct heapless::binary_heap::BinaryHeap\">BinaryHeap</a>&lt;T, K, N&gt; <span class=\"where fmt-newline\">where<br>&nbsp;&nbsp;&nbsp;&nbsp;T: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/cmp/trait.Ord.html\" title=\"trait core::cmp::Ord\">Ord</a>,<br>&nbsp;&nbsp;&nbsp;&nbsp;K: <a class=\"trait\" href=\"heapless/binary_heap/trait.Kind.html\" title=\"trait heapless::binary_heap::Kind\">Kind</a>,&nbsp;</span>","synthetic":false,"types":["heapless::binary_heap::BinaryHeap"]},{"text":"impl&lt;T, const N:&nbsp;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.64.0/core/primitive.usize.html\">usize</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a> for <a class=\"struct\" href=\"heapless/mpmc/struct.MpMcQueue.html\" title=\"struct heapless::mpmc::MpMcQueue\">MpMcQueue</a>&lt;T, N&gt;","synthetic":false,"types":["heapless::mpmc::MpMcQueue"]},{"text":"impl&lt;T, const N:&nbsp;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.64.0/core/primitive.usize.html\">usize</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a> for <a class=\"struct\" href=\"heapless/spsc/struct.Queue.html\" title=\"struct heapless::spsc::Queue\">Queue</a>&lt;T, N&gt;","synthetic":false,"types":["heapless::spsc::Queue"]}];
implementors["lock_api"] = [{"text":"impl&lt;R:&nbsp;<a class=\"trait\" href=\"lock_api/trait.RawMutex.html\" title=\"trait lock_api::RawMutex\">RawMutex</a>, T:&nbsp;?<a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/marker/trait.Sized.html\" title=\"trait core::marker::Sized\">Sized</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a> for <a class=\"struct\" href=\"lock_api/struct.Mutex.html\" title=\"struct lock_api::Mutex\">Mutex</a>&lt;R, T&gt;","synthetic":false,"types":["lock_api::mutex::Mutex"]},{"text":"impl&lt;R:&nbsp;<a class=\"trait\" href=\"lock_api/trait.RawMutex.html\" title=\"trait lock_api::RawMutex\">RawMutex</a>, G:&nbsp;<a class=\"trait\" href=\"lock_api/trait.GetThreadId.html\" title=\"trait lock_api::GetThreadId\">GetThreadId</a>, T:&nbsp;?<a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/marker/trait.Sized.html\" title=\"trait core::marker::Sized\">Sized</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a> for <a class=\"struct\" href=\"lock_api/struct.ReentrantMutex.html\" title=\"struct lock_api::ReentrantMutex\">ReentrantMutex</a>&lt;R, G, T&gt;","synthetic":false,"types":["lock_api::remutex::ReentrantMutex"]},{"text":"impl&lt;R:&nbsp;<a class=\"trait\" href=\"lock_api/trait.RawRwLock.html\" title=\"trait lock_api::RawRwLock\">RawRwLock</a>, T:&nbsp;?<a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/marker/trait.Sized.html\" title=\"trait core::marker::Sized\">Sized</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a> for <a class=\"struct\" href=\"lock_api/struct.RwLock.html\" title=\"struct lock_api::RwLock\">RwLock</a>&lt;R, T&gt;","synthetic":false,"types":["lock_api::rwlock::RwLock"]}];
implementors["micromath"] = [{"text":"impl <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a> for <a class=\"struct\" href=\"micromath/struct.F32.html\" title=\"struct micromath::F32\">F32</a>","synthetic":false,"types":["micromath::float::F32"]}];
implementors["spin"] = [{"text":"impl&lt;T:&nbsp;<a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a>, R&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a> for <a class=\"struct\" href=\"spin/lazy/struct.Lazy.html\" title=\"struct spin::lazy::Lazy\">Lazy</a>&lt;T, <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.64.0/core/primitive.fn.html\">fn</a>() -&gt; T, R&gt;","synthetic":false,"types":["spin::lazy::Lazy"]},{"text":"impl&lt;T:&nbsp;?<a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/marker/trait.Sized.html\" title=\"trait core::marker::Sized\">Sized</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a>, R&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a> for <a class=\"struct\" href=\"spin/mutex/spin/struct.SpinMutex.html\" title=\"struct spin::mutex::spin::SpinMutex\">SpinMutex</a>&lt;T, R&gt;","synthetic":false,"types":["spin::mutex::spin::SpinMutex"]},{"text":"impl&lt;T:&nbsp;?<a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/marker/trait.Sized.html\" title=\"trait core::marker::Sized\">Sized</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a>, R&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a> for <a class=\"struct\" href=\"spin/mutex/struct.Mutex.html\" title=\"struct spin::mutex::Mutex\">Mutex</a>&lt;T, R&gt;","synthetic":false,"types":["spin::mutex::Mutex"]},{"text":"impl&lt;T, R&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a> for <a class=\"struct\" href=\"spin/once/struct.Once.html\" title=\"struct spin::once::Once\">Once</a>&lt;T, R&gt;","synthetic":false,"types":["spin::once::Once"]},{"text":"impl&lt;T:&nbsp;?<a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/marker/trait.Sized.html\" title=\"trait core::marker::Sized\">Sized</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a>, R&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.64.0/core/default/trait.Default.html\" title=\"trait core::default::Default\">Default</a> for <a class=\"struct\" href=\"spin/rwlock/struct.RwLock.html\" title=\"struct spin::rwlock::RwLock\">RwLock</a>&lt;T, R&gt;","synthetic":false,"types":["spin::rwlock::RwLock"]}];
if (window.register_implementors) {window.register_implementors(implementors);} else {window.pending_implementors = implementors;}})()