# Notes

There will be all current things I wanna remind without having to open github or
smth else.

## TASKS

### TODO

- Rework physics architecture:
  - Replace `cDynamicBody::grounded` with 4-way contact flags (`top/bottom/left/right`).
  - Fix `DynamicBodySystem` (`force` scaled by `dt / mass` and reset, `impulse`
    divided by `mass`).

### IN PROGRESS

## KEEP FOR LATER

- Build new TileMap2D system, the last was based on an old 3d project, and it was
  spaghetti code.
- Simplify process to create an Animation (now: load atlas -> slice -> register)
- GPUMesh::transferToGpu() -> replace every frame allocation, by a persistent buffer
  for the transfer buffer.
- DynamicBatch -> reserve vertex and index buffer capacity before appending batch
  items.
- BatchHelper::appendCircle() -> optimize circle rendering (precomputed unit circle
  LUT or SDF quad shader instead of recalculating 24 cos/sin per circle every frame).
- Window -> wrap `SDL_Window*` in `unique_ptr<SDL_Window, SDLWindowDeleter>` and
  fix `window_count++` leak when `SDL_CreateWindow` fails.
- Renderer::createShader() / createMainGraphicsPipeline() -> return `unique_ptr`
  instead of raw owning pointers (`new` / `.release()`).
- Texture / GPUMesh / Shader::compileHLSL() -> wrap temp `SDL_Surface*`, transfer
  buffers, SPIR-V, and `SDL_GPUShader*` in RAII `unique_ptr` deleters.
- Game -> guard `~Game()` (`AssetManager::clear()`) against moved-from state or
  delete move ops.
- ActionMap::add() -> `std::move` `keys` and `mouse_btn` into `ActionBinding`
  (currently copies both vectors despite docs).
- Shader / MainGraphicsPipeline -> fix `noexcept` move ops that allocate or invoke
  reload callbacks (risks `std::terminate`).
- Animation / AnimationRegistry / MaterialRegistry -> add `&&` overloads to avoid
  copying frame and uniform vectors on registration.
- ResourceManager / AssetManager / AssetRegistry -> take `string_view` keys/scopes
  (zero-alloc cache hits) and `span<const string_view>` for multi-file loads.
- ComponentPool / ECSRegistry / StringArena / ActionMap -> return `std::span`
  instead of mutable `vector&` or static empty vectors (protects invariants).
- Texture / BitmapFont / Material / GPUMesh -> replace raw `(ptr, len)` and
  `void*+size` APIs with `std::span` (e.g. `span<const std::byte>`).
- GPUMesh::update() -> grow dynamic GPU buffer capacity via `std::bit_ceil` to
  reduce reallocation `SDL_WaitForGPUIdle` stalls.
- System<Components...> -> switch to CRTP `System<Derived, Components...>` to
  remove per-entity virtual dispatch in hot ECS/physics loops.
- ECSView -> add `if constexpr (sizeof...(Components) == 1)` fast path, default
  `Iterator()` + `enable_borrowed_range`, and drop redundant `operator!=`.
- ECS / AssetManager / AssetRegistry / Material -> constrain templates with C++20
  concepts (`Component`, `std::unsigned_integral`, `UniformBlock`, `invocable`).
- Event -> replace raw `SDL_Event` member with `std::variant` payload to remove
  `SDL_events.h` from `event.hpp` and make event getters type-safe.
- Core enums (`Key`, `EventType`, `EaseType`, etc.) -> add explicit underlying
  types (`uint8_t`/`uint16_t`) and use `using enum` in large switches.
- RaycastResult / Tile -> remove redundant `= default` ctors to restore C++20
  aggregates (designated initializers); consider `std::optional<RaycastHit>`.
- CircleShape::contains() / AABB2 / CircleCollider / Mat3 / Mat4 / Camera /
  UILayoutSystem -> mark pure math/collision helpers `constexpr` (drop `sqrt`).
- Point2 / Point3 -> use `operator<=> = default`; use `operator== = default` on
  `Vec2`/`Vec3`/`Vec4`.
- geometry/utils.hpp / easing.hpp / batch_helper.cpp / unit_mesh.cpp -> replace
  hardcoded `PI` with `std::numbers::pi_v<float>` + `inline constexpr` constants.
- MainGraphicsPipeline::createPipelineInternal() -> use `static constexpr array`
  for vertex attributes and designated initializers for SDL3 GPU descriptors.
- TimerManager::update() / ResourceManager::unloadScope() -> use `std::erase_if`
  (fixes `TimerManager` O(N^2) erase and callback re-entrancy).
- `[[nodiscard("reason")]]` & `[[unlikely]]` -> add reasons on leak/bug-prone
  APIs (`createEntity`, `beginFrame`, `poll`, `append`) and cold hot-path guards.
- `std::format` -> replace remaining `"..." + std::string(SDL_GetError())` chains
  in `.cpp` files (`window.cpp`, `render/gpu/`, `resource_manager.hpp`).
